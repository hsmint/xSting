#!/usr/bin/env python3
"""Offline conversion using urdf-usd-converter + OpenUSD (no GPU required)."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

import numpy as np
import yaml
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade
from urdf_usd_converter import Converter

PACKAGE = Path(__file__).resolve().parents[1]


def api(prim, name):
    # Author registered PhysX schema tokens without requiring a running Kit app.
    prim.AddAppliedSchema(name)


def attr(prim, name, kind, value):
    prim.CreateAttribute(name, kind, custom=False).Set(value)


def move(stage, source, target):
    editor = Usd.NamespaceEditor(stage)
    editor.MovePrimAtPath(source, target)
    assert editor.CanApplyEdits(), (source, target)
    assert editor.ApplyEdits()


def validate(path, urdf):
    stage = Usd.Stage.Open(str(path))
    bodies = {p.GetName(): p for p in stage.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)}
    joints = {p.GetName(): p for p in stage.Traverse() if p.IsA(UsdPhysics.Joint)}
    expected = {j.get('name'): j for j in urdf.findall('joint') if j.get('type') != 'fixed'}
    assert set(joints) == set(expected)
    assert len(bodies) == 26 and len(joints) == 25
    roots = [p for p in stage.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
    assert len(roots) == 1 and roots[0] == bodies['base']
    assert not any(p.IsA(UsdPhysics.FixedJoint) for p in stage.Traverse())
    mass = 0
    for link in urdf.findall('link'):
        inertia = link.find('inertial')
        if inertia is None:
            assert any(p.GetName() == link.get('name') for p in stage.Traverse()), link.get('name')
            continue
        p = bodies[link.get('name')]
        m = UsdPhysics.MassAPI(p)
        assert np.isclose(m.GetMassAttr().Get(), float(inertia.find('mass').get('value')))
        eig = np.array(m.GetDiagonalInertiaAttr().Get())
        assert (eig > 0).all() and 2 * eig.max() <= eig.sum() + 1e-8
        mass += m.GetMassAttr().Get()
        assert p.GetParent().GetPath() == Sdf.Path('/xsting')
    connected = {'base'}
    cache = UsdGeom.XformCache()
    for name, node in expected.items():
        j = UsdPhysics.Joint(joints[name])
        parent, child = node.find('parent').get('link'), node.find('child').get('link')
        assert j.GetBody0Rel().GetTargets() == [bodies[parent].GetPath()]
        assert j.GetBody1Rel().GetTargets() == [bodies[child].GetPath()]
        limit = node.find('limit')
        factor = 180 / math.pi if node.get('type') == 'revolute' else 1
        for key in ['lower','upper']:
            assert np.isclose(j.GetPrim().GetAttribute('physics:' + key + 'Limit').Get(), float(limit.get(key))*factor)
        # Zero-pose joint frames must coincide in world space, including signed axes.
        frames = []
        for body, pos, rot in [(bodies[parent],j.GetLocalPos0Attr().Get(),j.GetLocalRot0Attr().Get()),
                               (bodies[child],j.GetLocalPos1Attr().Get(),j.GetLocalRot1Attr().Get())]:
            frame = Gf.Matrix4d(1).SetRotate(Gf.Quatd(rot))
            frame.SetTranslateOnly(Gf.Vec3d(pos))
            frames.append(frame * cache.GetLocalToWorldTransform(body))
        assert np.allclose(np.array(frames[0]), np.array(frames[1]), atol=1e-6), name
    while True:
        reached = connected | {j.find('child').get('link') for j in expected.values() if j.find('parent').get('link') in connected}
        if reached == connected: break
        connected = reached
    assert connected == set(bodies)
    mimic = joints['finger_r']
    assert mimic.GetRelationship('physxMimicJoint:rotX:referenceJoint').GetTargets() == [joints['finger_l'].GetPath()]
    assert mimic.GetAttribute('physxMimicJoint:rotX:gearing').Get() == -1
    collision_count = 0
    for p in Usd.PrimRange(stage.GetPseudoRoot(), Usd.TraverseInstanceProxies()):
        if p.HasAPI(UsdPhysics.CollisionAPI):
            collision_count += 1
            assert UsdPhysics.MeshCollisionAPI(p).GetApproximationAttr().Get() == 'convexDecomposition'
    assert collision_count == 26
    # Reference under another prim to catch non-portable paths and broken prototypes.
    test = Usd.Stage.CreateInMemory()
    test.DefinePrim('/World/Robot').GetReferences().AddReference(str(path.resolve()))
    assert sum(p.HasAPI(UsdPhysics.RigidBodyAPI) for p in test.Traverse()) == 26
    assert sum(p.IsInstance() for p in test.Traverse()) >= 52
    for p in test.Traverse():
        if p.IsA(UsdShade.Material):
            assert UsdShade.Material(p).ComputeSurfaceSource()[0], p.GetPath()
    return dict(rigid_bodies=26, moving_joints=25, independent_actions=24,
                collision_meshes=collision_count, mass_kg=mass, structural_validation='PASS',
                runtime_validation='NOT RUN: NVIDIA GPU unavailable')


def build(out):
    out.mkdir(parents=True, exist_ok=True)
    urdf = ET.parse(PACKAGE/'urdf/xsting.urdf').getroot()
    source = PACKAGE/'urdf/xsting.urdf'
    with tempfile.TemporaryDirectory(prefix='xsting_convert_') as temp:
        converted = Converter(layer_structure=False, scene=False,
            ros_packages=[{'name':'xsting_description','path':str(PACKAGE)}]).convert(str(source),temp)
        stage = Usd.Stage.Open(converted.path)
        # Keep all prototype/material dependencies inside the referenceable default prim.
        UsdGeom.Scope.Define(stage, '/xsting/Library')
        for p in list(stage.GetPseudoRoot().GetAllChildren()):
            if p.GetName() != 'xsting': move(stage,p.GetPath(),'/xsting/Library/'+p.GetName())
        stage.RemovePrim('/xsting/Physics/root_joint')  # mobile robot: no world anchor
        cache = UsdGeom.XformCache()
        bodies = [p for p in stage.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)]
        transforms = {str(p.GetPath()):cache.GetLocalToWorldTransform(p) for p in bodies}
        for old in sorted(transforms, key=lambda p:p.count('/'), reverse=True):
            target = '/xsting/' + old.rsplit('/',1)[1]
            move(stage,old,target)
            xform = UsdGeom.Xformable(stage.GetPrimAtPath(target))
            xform.ClearXformOpOrder()
            xform.AddTransformOp().Set(transforms[old])
        stage.RemovePrim('/xsting/Geometry')
        root = stage.GetPrimAtPath('/xsting/base')
        api(root,'PhysxArticulationAPI')
        attr(root,'physxArticulation:enabledSelfCollisions',Sdf.ValueTypeNames.Bool,False)
        attr(root,'physxArticulation:solverPositionIterationCount',Sdf.ValueTypeNames.Int,8)
        attr(root,'physxArticulation:solverVelocityIterationCount',Sdf.ValueTypeNames.Int,2)
        # Explicit PhysX drives: torque control by default; training chooses actuator gains.
        for node in urdf.findall('joint'):
            if node.get('type') == 'fixed': continue
            p = stage.GetPrimAtPath('/xsting/Physics/'+node.get('name'))
            angular = node.get('type') == 'revolute'
            limit = node.find('limit')
            api(p,'PhysxJointAPI')
            attr(p,'physxJoint:maxJointVelocity',Sdf.ValueTypeNames.Float,
                 float(limit.get('velocity')) * (180/math.pi if angular else 1))
            if node.find('mimic') is None:
                d = UsdPhysics.DriveAPI.Apply(p,'angular' if angular else 'linear')
                d.CreateTypeAttr('force')
                d.CreateStiffnessAttr(0)
                d.CreateDampingAttr(0)
                d.CreateMaxForceAttr(float(limit.get('effort')))
            else:
                api(p,'PhysxMimicJointAPI:rotX')
                p.CreateRelationship('physxMimicJoint:rotX:referenceJoint').SetTargets(['/xsting/Physics/finger_l'])
                attr(p,'physxMimicJoint:rotX:referenceJointAxis',Sdf.ValueTypeNames.Token,'rotX')
                attr(p,'physxMimicJoint:rotX:gearing',Sdf.ValueTypeNames.Float,-1)
                attr(p,'physxMimicJoint:rotX:offset',Sdf.ValueTypeNames.Float,0)
        # Remove Newton-only semantics so this asset has one unambiguous PhysX setup.
        for p in stage.Traverse():
            for name in list(p.GetMetadata('apiSchemas').GetAppliedItems() if p.HasMetadata('apiSchemas') else []):
                if name.startswith('Newton'): p.RemoveAppliedSchema(name)
            for prop in list(p.GetProperties()):
                if prop.GetName().startswith('newton:'): p.RemoveProperty(prop.GetName())
        # Reopen at the destination before authoring relative mesh references.
        output = out/'xsting_training.usda'
        stage.GetRootLayer().Export(str(output))
        stage = Usd.Stage.Open(str(output))
        # External instanceable geometry prevents ASCII mesh bloat and shares geometry across envs.
        library = Usd.Stage.CreateNew(str(out/'xsting_meshes.usdc'))
        mesh_prims = [p for p in stage.Traverse() if p.IsA(UsdGeom.Mesh)]
        for i,p in enumerate(mesh_prims):
            path = p.GetPath()
            item = '/Mesh_' + str(i)
            UsdGeom.Xform.Define(library,item)
            Sdf.CopySpec(stage.GetRootLayer(),path,library.GetRootLayer(),item+'/Shape')
            shape = library.GetPrimAtPath(item+'/Shape')
            bindings = [(r.GetName(),r.GetTargets()) for r in p.GetRelationships() if r.GetName().startswith('material:binding')]
            for name,_ in bindings: shape.RemoveProperty(name)
            if shape.HasAPI(UsdPhysics.CollisionAPI):
                UsdPhysics.MeshCollisionAPI.Apply(shape).CreateApproximationAttr('convexDecomposition')
                api(shape,'PhysxCollisionAPI')
                attr(shape,'physxCollision:contactOffset',Sdf.ValueTypeNames.Float,0.001)
                attr(shape,'physxCollision:restOffset',Sdf.ValueTypeNames.Float,0)
            stage.RemovePrim(path)
            wrapper = UsdGeom.Xform.Define(stage,path).GetPrim()
            wrapper.GetReferences().AddReference('./xsting_meshes.usdc',item)
            for name,targets in bindings: wrapper.CreateRelationship(name).SetTargets(targets)
            if bindings: api(wrapper,'MaterialBindingAPI')
            wrapper.SetInstanceable(True)
        library.GetRootLayer().Save()
        stage.GetRootLayer().documentation = 'xSting PhysX training asset; source: ../urdf/xsting.urdf. Zero pose; floating base; zero drive gains.'
        stage.GetRootLayer().Save()
    order = yaml.safe_load((PACKAGE/'config/joint_order.yaml').read_text())
    ordered = order['leg_joints'] + order['arm_joints']
    settings = {'usd':'xsting_training.usda','articulation_root':'/xsting/base',
                'action_joint_names':[j['name'] for j in ordered],
                'initial_joint_positions':{j['name']:j['default'] for j in ordered},
                'initial_base_position':[0,0,0.04], 'units':'m, kg, rad (training API)',
                'passive_mimic_joint':'finger_r','drive_stiffness':0,'drive_damping':0,
                'note':'Set/reset the stand pose from initial_joint_positions in your training environment. Clamp applied torques to URDF effort limits.'}
    settings['initial_joint_positions']['finger_r']=0
    (out/'training_config.json').write_text(json.dumps(settings,indent=2)+'\n')
    report = validate(output,urdf)
    report['urdf_sha256'] = hashlib.sha256(source.read_bytes()).hexdigest()
    (out/'validation.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=PACKAGE/'usd')
    args = parser.parse_args()
    build(args.output.resolve())
