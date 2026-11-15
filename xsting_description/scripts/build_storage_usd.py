#!/usr/bin/env python3
"""Build storage-specific precision colliders and test 30 mm pucks against CAD.
Requires OpenUSD, numpy, trimesh, scipy and python-fcl. All units are metres.
"""
from pathlib import Path
import json, math, hashlib
import xml.etree.ElementTree as ET
import numpy as np
import trimesh
import fcl
from scipy.spatial.transform import Rotation
from pxr import Usd, UsdGeom, UsdPhysics, UsdLux, Sdf, Gf, Vt

PACKAGE=Path(__file__).resolve().parents[1]
OUT=PACKAGE/'usd/storage'

def attr(p,n,t,v):p.CreateAttribute(n,t,custom=False).Set(v)
def mesh(stage,path,v,f,collision=False,approx='convexDecomposition'):
    m=UsdGeom.Mesh.Define(stage,path)
    m.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(np.asarray(v,dtype=np.float32)))
    m.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(np.full(len(f),3,dtype=np.int32)))
    m.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(np.asarray(f,dtype=np.int32).ravel()))
    m.CreateSubdivisionSchemeAttr('none')
    m.CreateExtentAttr([Gf.Vec3f(*np.min(v,axis=0)),Gf.Vec3f(*np.max(v,axis=0))])
    if collision:
        UsdPhysics.CollisionAPI.Apply(m.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim()).CreateApproximationAttr(approx)
        m.CreatePurposeAttr('guide')
        m.GetPrim().AddAppliedSchema('PhysxCollisionAPI')
        attr(m.GetPrim(),'physxCollision:contactOffset',Sdf.ValueTypeNames.Float,.0002)
        attr(m.GetPrim(),'physxCollision:restOffset',Sdf.ValueTypeNames.Float,0)
    return m

def bvh(m):
    b=fcl.BVHModel();b.beginModel(len(m.vertices),len(m.faces));b.addSubModel(m.vertices,m.faces);b.endModel()
    return fcl.CollisionObject(b)

def generate():
    OUT.mkdir(exist_ok=True)
    original=PACKAGE/'meshes/body/base_link.stl'
    body=trimesh.load(original,force='mesh');body.apply_scale(.001)
    parts=body.split(only_watertight=False)
    candidates=[s for s in parts if s.is_watertight and len(s.faces)>50000 and s.bounds[0,2]>.16]
    assert len(candidates)==1,'Source geometry changed: identify the basket again'
    basket=candidates[0]
    cut=basket.bounds[0,2]
    # Four physical waffle decks, bottom to top. Replace their detailed holes and
    # intervening chassis hardware with deck-sized box collision slabs.
    decks={}
    for part in parts:
        extent=part.extents
        if part.is_watertight and abs(extent[2]-.009)<1e-5 and min(extent[:2])>.07:
            decks.setdefault(round(float(part.bounds[0,2]),6),[]).append(part)
    assert len(decks)==4,'Expected four waffle-deck levels'
    upper=trimesh.util.concatenate([s for s in parts if s is not basket and s.bounds[1,2]>cut+1e-6])
    # Front RealSense is a separate protruding assembly in the source base STL.
    # Its outer convex envelope preserves the rounded housing without retaining
    # internal electronics or allowing contacts inside the camera housing.
    camera_parts=[s for s in parts if s.bounds[0,0]>.136 and s.bounds[1,0]<.163
        and s.bounds[0,2]>.094 and s.bounds[1,2]<.121
        and s.bounds[0,1]>-.046 and s.bounds[1,1]<.046]
    assert camera_parts,'Front RealSense source assembly not found'
    camera_source=trimesh.util.concatenate(camera_parts)
    camera_hull=camera_source.convex_hull
    assert .089 < camera_hull.extents[1] < .091 and .024 < camera_hull.extents[2] < .026
    library=Usd.Stage.CreateNew(str(OUT/'storage_colliders.usdc'))
    UsdGeom.Xform.Define(library,'/StorageColliders')
    floor_boxes=[]
    for index,(_,sections) in enumerate(sorted(decks.items()),1):
        bounds=np.array([s.bounds for s in sections])
        low=bounds[:,0].min(axis=0);high=bounds[:,1].max(axis=0)
        cube=UsdGeom.Cube.Define(library,f'/StorageColliders/Floor{index}')
        cube.CreateSizeAttr(1)
        cube.AddTranslateOp().Set(Gf.Vec3d(*((low+high)/2)))
        cube.AddScaleOp().Set(Gf.Vec3f(*(high-low)))
        cube.CreatePurposeAttr('guide')
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        cube.GetPrim().AddAppliedSchema('PhysxCollisionAPI')
        attr(cube.GetPrim(),'physxCollision:contactOffset',Sdf.ValueTypeNames.Float,.0002)
        attr(cube.GetPrim(),'physxCollision:restOffset',Sdf.ValueTypeNames.Float,0)
        floor_boxes.append({'floor':index,'min_m':low.tolist(),'max_m':high.tolist()})
    for name,part in [('Basket',basket),('ArmMount',upper)]:
        m=mesh(library,'/StorageColliders/'+name,part.vertices,part.faces,True,'sdf' if name=='Basket' else 'convexDecomposition')
        p=m.GetPrim()
        if name=='Basket':
            p.AddAppliedSchema('PhysxSDFMeshCollisionAPI')
            attr(p,'physxSDFMeshCollision:sdfResolution',Sdf.ValueTypeNames.Int,1024)
            attr(p,'physxSDFMeshCollision:sdfSubgridResolution',Sdf.ValueTypeNames.Int,6)
            attr(p,'physxSDFMeshCollision:sdfBitsPerSubgridPixel',Sdf.ValueTypeNames.Token,'BitsPerPixel16')
        else:
            p.AddAppliedSchema('PhysxConvexDecompositionCollisionAPI')
            attr(p,'physxConvexDecompositionCollision:maxConvexHulls',Sdf.ValueTypeNames.Int,128)
            attr(p,'physxConvexDecompositionCollision:shrinkWrap',Sdf.ValueTypeNames.Bool,True)
    front=mesh(library,'/StorageColliders/FrontRealSense',camera_hull.vertices,camera_hull.faces,True,'convexHull').GetPrim()
    front.AddAppliedSchema('PhysxConvexHullCollisionAPI')
    attr(front,'physxConvexHullCollision:hullVertexLimit',Sdf.ValueTypeNames.Int,256)
    library.SetDefaultPrim(library.GetPrimAtPath('/StorageColliders'))
    UsdGeom.SetStageMetersPerUnit(library,1.0);UsdGeom.SetStageUpAxis(library,'Z')
    library.GetRootLayer().Save()
    # Separate presets preserve the original RL and manipulation assets.
    for kind in ['training','manipulation']:
        stage=Usd.Stage.CreateNew(str(PACKAGE/f'usd/xsting_storage_{kind}.usda'))
        root=UsdGeom.Xform.Define(stage,'/xsting').GetPrim()
        root.GetReferences().AddReference(f'./xsting_{kind}.usda');stage.SetDefaultPrim(root)
        UsdGeom.SetStageMetersPerUnit(stage,1.0);UsdGeom.SetStageUpAxis(stage,'Z')
        stage.SetStartTimeCode(0);stage.SetEndTimeCode(1000000000);stage.SetTimeCodesPerSecond(60)
        old=stage.GetPrimAtPath('/xsting/base/base_link_1')
        assert old and old.IsInstance()
        old.SetActive(False)
        new=stage.DefinePrim('/xsting/base/StorageColliders','Xform')
        new.GetReferences().AddReference('./storage/storage_colliders.usdc');new.SetInstanceable(True)
        root.SetCustomDataByKey('storage:nominalPuckSize',Gf.Vec3d(.03,.03,.06))
        from build_sensor_usd import add_sensors
        add_sensors(stage)
        stage.GetRootLayer().Save()
    slots=[]
    for j in ET.parse(PACKAGE/'urdf/xsting.urdf').getroot().findall('joint'):
        if j.get('name','').startswith('pallet_'):
            slots.append({'name':j.find('child').get('link'),
                'xyz':list(map(float,j.find('origin').get('xyz').split())),
                'yaw':float(j.find('origin').get('rpy').split()[2])})
    exact=bvh(basket)
    results=[]
    for slot in slots:
        for kind,geometry in [('square',fcl.Box(.03,.03,.06)),('round',fcl.Cylinder(.015,.06))]:
            rotation=Rotation.from_euler('z',slot['yaw']).as_matrix()
            xyz=np.array(slot['xyz']);puck=fcl.CollisionObject(geometry)
            def collided(bottom):
                pos=xyz.copy();pos[2]=bottom+.03
                puck.setTransform(fcl.Transform(rotation,pos))
                return bool(fcl.collide(exact,puck,fcl.CollisionRequest(),fcl.CollisionResult()))
            # Continuous vertical route is sampled at 0.5 mm; source support is then
            # located by bisection, independently of USD/SDF collision cooking.
            bottom_values=np.arange(basket.bounds[1,2]+.005,xyz[2]-.003,-.0005)
            first=next((i for i,z in enumerate(bottom_values) if collided(z)),None)
            assert first is not None and first>0,(slot['name'],kind,'no support found')
            lo,hi=bottom_values[first],bottom_values[first-1]
            for _ in range(24):
                mid=(lo+hi)/2
                if collided(mid):lo=mid
                else:hi=mid
            support=hi
            assert support < xyz[2]+.002,(slot['name'],kind,'blocked above slot')
            results.append({'slot':slot['name'],'shape':kind,'support_bottom_z':support,
                'frame_to_support_mm':(support-xyz[2])*1000,
                'vertical_insertion_samples':first,'source_mesh_fit':'PASS'})
    report={'source_sha256':hashlib.sha256(original.read_bytes()).hexdigest(),
        'basket_watertight':bool(basket.is_watertight),'basket_triangles':len(basket.faces),'floor_boxes':floor_boxes,
        'front_realsense':{'approximation':'convex outer envelope of source CAD assembly',
            'source_parts':len(camera_parts),'hull_vertices':len(camera_hull.vertices),
            'min_m':camera_hull.bounds[0].tolist(),'max_m':camera_hull.bounds[1].tolist(),
            'size_mm':(camera_hull.extents*1000).tolist()},
        'sdf_resolution':1024,'nominal_sdf_cell_mm':float(max(basket.extents)/1024*1000),
        'contact_offset_mm':.2,'square_size_mm':[30,30,60],'round_diameter_height_mm':[30,60],
        'round_collision_segments':32,'round_radial_approximation_error_mm':15*(1-math.cos(math.pi/32)),
        'support_tests':results,'runtime_physics':'See runtime_validation.json; rerun after regenerating assets',
        'puck_test_density_kg_m3':1000,'mass_note':'Test assumption, not measured puck mass'}
    (OUT/'geometry_validation.json').write_text(json.dumps(report,indent=2)+'\n')
    (OUT/'slots.json').write_text(json.dumps(slots,indent=2)+'\n')
    for kind in ['square','round']:
        stage=Usd.Stage.CreateNew(str(OUT/f'inspect_{kind}.usda'))
        world=UsdGeom.Xform.Define(stage,'/World');stage.SetDefaultPrim(world.GetPrim())
        UsdGeom.SetStageMetersPerUnit(stage,1.0);UsdGeom.SetStageUpAxis(stage,'Z')
        stage.SetStartTimeCode(0);stage.SetEndTimeCode(1000000000);stage.SetTimeCodesPerSecond(60)
        stage.DefinePrim('/World/xsting','Xform').GetReferences().AddReference('../xsting_storage_manipulation.usda')
        scene=UsdPhysics.Scene.Define(stage,'/World/PhysicsScene');scene.CreateGravityMagnitudeAttr(9.81)
        scene.CreateGravityDirectionAttr(Gf.Vec3f(0,0,-1))
        scene.GetPrim().AddAppliedSchema('PhysxSceneAPI')
        for name,t,value in [('enableGPUDynamics',Sdf.ValueTypeNames.Bool,True),('broadphaseType',Sdf.ValueTypeNames.Token,'GPU'),('solverType',Sdf.ValueTypeNames.Token,'TGS'),('timeStepsPerSecond',Sdf.ValueTypeNames.Int,240)]:
            attr(scene.GetPrim(),'physxScene:'+name,t,value)
        UsdLux.DomeLight.Define(stage,'/World/Light').CreateIntensityAttr(1200)
        cam=UsdGeom.Camera.Define(stage,'/World/Camera');cam.CreateClippingRangeAttr(Gf.Vec2f(.001,100))
        cam.CreateFocalLengthAttr(28)
        cam.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(.48,-.6,.65),Gf.Vec3d(-.01,0,.22),Gf.Vec3d(0,0,1)).GetInverse())
        for slot in slots:
            item=f'/World/Pucks/{slot["name"]}'
            record=next(r for r in results if r['slot']==slot['name'] and r['shape']==kind)
            xf=UsdGeom.Xform.Define(stage,item)
            xf.AddTranslateOp().Set(Gf.Vec3d(*slot['xyz'][:2],record['support_bottom_z']+.03+.04+.005))
            xf.AddOrientOp().Set(Gf.Quatf(math.cos(slot['yaw']/2),0,0,math.sin(slot['yaw']/2)))
            if kind=='square':
                geom=UsdGeom.Cube.Define(stage,item+'/Shape');geom.CreateSizeAttr(1)
                geom.AddScaleOp().Set(Gf.Vec3f(.03,.03,.06));mass=.03*.03*.06*1000
            else:
                cylinder=trimesh.creation.cylinder(radius=.015,height=.06,sections=32)
                geom=mesh(stage,item+'/Shape',cylinder.vertices,cylinder.faces)
                UsdPhysics.MeshCollisionAPI.Apply(geom.GetPrim()).CreateApproximationAttr('convexHull')
                mass=math.pi*.015**2*.06*1000
            geom.CreateDisplayColorAttr([Gf.Vec3f(.05,.55,.9) if kind=='square' else Gf.Vec3f(.95,.45,.05)])
            UsdPhysics.CollisionAPI.Apply(geom.GetPrim());geom.GetPrim().AddAppliedSchema('PhysxCollisionAPI')
            attr(geom.GetPrim(),'physxCollision:contactOffset',Sdf.ValueTypeNames.Float,.0002)
            attr(geom.GetPrim(),'physxCollision:restOffset',Sdf.ValueTypeNames.Float,0)
            UsdPhysics.RigidBodyAPI.Apply(xf.GetPrim());UsdPhysics.MassAPI.Apply(xf.GetPrim()).CreateMassAttr(mass)
            xf.GetPrim().AddAppliedSchema('PhysxRigidBodyAPI')
            attr(xf.GetPrim(),'physxRigidBody:solverPositionIterationCount',Sdf.ValueTypeNames.Int,32)
            attr(xf.GetPrim(),'physxRigidBody:solverVelocityIterationCount',Sdf.ValueTypeNames.Int,8)
        stage.GetRootLayer().Save()
    print('STORAGE_ASSETS_READY',len(results),'source-mesh insertion/support tests passed')

if __name__=='__main__':generate()
