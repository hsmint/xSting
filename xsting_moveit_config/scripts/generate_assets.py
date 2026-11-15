#!/usr/bin/env python3
"""Generate a stationary manipulation model, SRDF, XRDF and Isaac Sim overlay."""
from pathlib import Path
import itertools
import json
import math
import xml.etree.ElementTree as ET
import numpy as np
import trimesh
import yaml
from scipy.spatial.transform import Rotation
from pxr import Usd, UsdGeom, UsdPhysics, Gf, Sdf

ROOT = Path(__file__).resolve().parents[2]
DESC = ROOT/'xsting_description'
OUT = ROOT/'xsting_moveit_config/config'
ARM = ['arm_yaw','arm_shoulder','arm_elbow','arm_wrist','arm_roll']

def dump(name, data):
    (OUT/name).write_text(yaml.safe_dump(data, sort_keys=False))

def origin(j):
    o = j.find('origin')
    t = np.eye(4)
    if o is not None:
        t[:3,3] = np.fromstring(o.get('xyz','0 0 0'),sep=' ')
        t[:3,:3] = Rotation.from_euler('xyz',np.fromstring(o.get('rpy','0 0 0'),sep=' ')).as_matrix()
    return t

def write_xml(name, root):
    ET.indent(root)
    ET.ElementTree(root).write(OUT/name, encoding='utf-8',xml_declaration=True)

def generate():
    robot = ET.parse(DESC/'urdf/xsting.urdf').getroot()
    cfg = yaml.safe_load((DESC/'config/joint_order.yaml').read_text())
    defaults = {j['name']:float(j['default']) for j in cfg['leg_joints']+cfg['arm_joints']}
    # The folded walking pose is mesh-collision-free but rejected by conservative
    # sphere geometry near the shoulder. Start manipulation in the open L-pose.
    defaults.update({name:0.0 for name in ARM})
    defaults['finger_r'] = 0.0
    # Bake stationary legs into fixed URDF joints while retaining collision geometry.
    for j in robot.findall('joint'):
        if j.get('name').startswith('leg'):
            t = origin(j)
            axis = np.fromstring(j.find('axis').get('xyz'),sep=' ')
            t[:3,:3] = t[:3,:3] @ Rotation.from_rotvec(axis*defaults[j.get('name')]).as_matrix()
            j.find('origin').set('rpy',' '.join(map(str,Rotation.from_matrix(t[:3,:3]).as_euler('xyz'))))
            j.set('type','fixed')
            for tag in ['axis','limit','dynamics']:
                child=j.find(tag)
                if child is not None: j.remove(child)
    # TCP is the midpoint between the finger reference origins, not a calibrated grasp contact.
    ET.SubElement(robot,'link',name='tool0')
    tcp=ET.SubElement(robot,'joint',name='tool0_joint',type='fixed')
    ET.SubElement(tcp,'parent',link='gripper'); ET.SubElement(tcp,'child',link='tool0')
    ET.SubElement(tcp,'origin',xyz='0.0995 0 0',rpy='0 0 0')
    for sensor in robot.findall('sensor'): robot.remove(sensor)
    write_xml('xsting_manipulation.urdf',robot)
    joints=robot.findall('joint')
    physical=[l.get('name') for l in robot.findall('link') if l.find('collision') is not None]
    fixed=['base']+[n for n in physical if n.startswith('leg')]
    ignore=set(tuple(sorted(pair)) for pair in itertools.combinations(fixed,2))
    for j in joints:
        a,b=j.find('parent').get('link'),j.find('child').get('link')
        if a in physical and b in physical: ignore.add(tuple(sorted((a,b))))
    ignore.add(('finger_l','finger_r'))
    # The gripper fingers mount through the wrist housing; these pairs overlap by construction.
    for n in ['finger_l','finger_r']: ignore.add(tuple(sorted(('arm_4',n))))
    srdf=ET.Element('robot',name='xsting')
    arm=ET.SubElement(srdf,'group',name='arm'); ET.SubElement(arm,'chain',base_link='base',tip_link='tool0')
    grip=ET.SubElement(srdf,'group',name='gripper')
    for n in ['finger_l','finger_r']: ET.SubElement(grip,'joint',name=n)
    ET.SubElement(srdf,'end_effector',name='hand',parent_link='gripper',group='gripper',parent_group='arm')
    for name in ['home','zero']:
        state=ET.SubElement(srdf,'group_state',name=name,group='arm')
        for n in ARM: ET.SubElement(state,'joint',name=n,value=str(defaults[n] if name=='home' else 0.0))
    for name,q in [('open',0.025),('closed',0.0)]:
        state=ET.SubElement(srdf,'group_state',name=name,group='gripper')
        ET.SubElement(state,'joint',name='finger_l',value=str(q))
    for a,b in sorted(ignore): ET.SubElement(srdf,'disable_collisions',link1=a,link2=b,reason='Adjacent' if (a not in fixed or b not in fixed) else 'Fixed')
    write_xml('xsting.srdf',srdf)
    spheres={}; counts={}
    # Each triangle is fully contained in its assigned sphere (a convex sphere contains
    # all three vertices). Grid grouping reduces count without random sampling.
    for link in robot.findall('link'):
        col=link.find('collision')
        if col is None: continue
        mesh=col.find('geometry/mesh')
        file=DESC/mesh.get('filename').split('package://xsting_description/')[1]
        tri=np.array(trimesh.load(file,force='mesh',process=False).triangles)
        tri*=np.fromstring(mesh.get('scale','1 1 1'),sep=' ')
        t=origin(col); tri=tri@t[:3,:3].T+t[:3,3]
        size=0.006 if link.get('name') in ['base','arm_2'] else 0.012
        # CAD triangles can be much longer than the grid cell. Subdivide them
        # before grouping so a single long triangle cannot inflate a whole sphere.
        vertices,faces=trimesh.remesh.subdivide_to_size(
            tri.reshape(-1,3),np.arange(tri.size//3).reshape(-1,3),max_edge=size,max_iter=10)
        tri=vertices[faces]
        centroids=tri.mean(axis=1)
        sizes=np.full(len(tri),size)
        if link.get('name')=='base':
            # Keep fine coverage only near the shoulder mount; the stationary
            # chassis can use coarser cells without burdening every GPU query.
            near_mount=(centroids[:,0]>0.05)&(centroids[:,2]>0.17)&(abs(centroids[:,1])<0.08)
            sizes[~near_mount]=0.025
        cells=np.floor(centroids/sizes[:,None]).astype(int)
        keys,inverse=np.unique(np.column_stack((sizes,cells)),axis=0,return_inverse=True)
        centers=(keys[:,1:]+0.5)*keys[:,:1]
        radii=np.zeros(len(keys))
        np.maximum.at(radii,inverse,np.linalg.norm(tri-centers[inverse,None,:],axis=2).max(axis=1))
        # Fingers are locked in XRDF. Cover the full commanded stroke conservatively.
        pad=0.027 if link.get('name').startswith('finger') else 0.001
        radii+=pad
        assert np.all(np.linalg.norm(tri-centers[inverse,None,:],axis=2) <= radii[inverse,None]+1e-10)
        spheres[link.get('name')]=[{'center':[round(float(v),7) for v in c], 'radius':math.ceil(float(r)*1e7)/1e7} for c,r in zip(centers,radii)]
        counts[link.get('name')]=len(keys)
    ignores={}
    for a,b in sorted(ignore): ignores.setdefault(a,[]).append(b)
    xrdf={'format':'xrdf','format_version':1.0,'modifiers':[{'set_base_frame':'base'}],
          'default_joint_positions':{n:defaults[n] for n in ARM+['finger_l','finger_r']},
          'cspace':{'joint_names':ARM,'acceleration_limits':[2.0]*5,'jerk_limits':[20.0]*5},
          'tool_frames':['tool0'],'collision':{'geometry':'robot_spheres'},
          'self_collision':{'geometry':'robot_spheres','ignore':ignores},
          'geometry':{'robot_spheres':{'spheres':spheres}}}
    dump('xsting.xrdf',xrdf)
    limits={}
    for j in joints:
        if j.get('type')=='fixed': continue
        v=float(j.find('limit').get('velocity'))
        limits[j.get('name')]={'has_velocity_limits':True,'max_velocity':min(v,1.0),
          'has_acceleration_limits':True,'max_acceleration':0.1 if j.get('type')=='prismatic' else 2.0}
    dump('joint_limits.yaml',{'default_velocity_scaling_factor':0.2,'default_acceleration_scaling_factor':0.2,'joint_limits':limits})
    # A reference overlay leaves the RL asset's floating base and torque drives intact.
    path=DESC/'usd/xsting_manipulation.usda'
    stage=Usd.Stage.CreateNew(str(path))
    root=UsdGeom.Xform.Define(stage,'/xsting').GetPrim()
    root.GetReferences().AddReference('./xsting_training.usda')
    stage.SetDefaultPrim(root); UsdGeom.SetStageUpAxis(stage,'Z'); UsdGeom.SetStageMetersPerUnit(stage,1.0)
    source_joints={j.get('name'):j for j in ET.parse(DESC/'urdf/xsting.urdf').getroot().findall('joint')}
    transforms={'base':np.eye(4)}; transforms['base'][2,3]=0.04
    pending=list(source_joints.values())
    while pending:
        for j in pending[:]:
            parent=j.find('parent').get('link'); child=j.find('child').get('link')
            if parent not in transforms: continue
            t=origin(j); q=defaults.get(j.get('name'),0)
            if j.get('type')=='revolute':
                t[:3,:3]=t[:3,:3]@Rotation.from_rotvec(np.fromstring(j.find('axis').get('xyz'),sep=' ')*q).as_matrix()
            transforms[child]=transforms[parent]@t; pending.remove(j)
    for p in list(stage.Traverse()):
        if p.HasAPI(UsdPhysics.RigidBodyAPI):
            UsdGeom.Xformable(p).GetOrderedXformOps()[0].Set(Gf.Matrix4d(transforms[p.GetName()].T.tolist()))
    anchor=UsdPhysics.FixedJoint.Define(stage,'/xsting/Physics/world_anchor')
    anchor.CreateBody1Rel().SetTargets(['/xsting/base'])
    anchor.CreateLocalPos0Attr(Gf.Vec3f(0,0,0.04))
    base=stage.GetPrimAtPath('/xsting/base')
    base.RemoveAPI(UsdPhysics.ArticulationRootAPI)
    UsdPhysics.ArticulationRootAPI.Apply(anchor.GetPrim())
    base.RemoveAppliedSchema('PhysxArticulationAPI')
    anchor.GetPrim().AddAppliedSchema('PhysxArticulationAPI')
    anchor.GetPrim().CreateAttribute('physxArticulation:enabledSelfCollisions',Sdf.ValueTypeNames.Bool).Set(False)
    for name,q in defaults.items():
        j=stage.GetPrimAtPath('/xsting/Physics/'+name)
        if name.startswith('leg'):
            # Preserve DOF names for ROS joint state interoperability, but lock their ranges.
            j.GetAttribute('physics:lowerLimit').Set(math.degrees(q)); j.GetAttribute('physics:upperLimit').Set(math.degrees(q))
        if name=='finger_r': continue
        angular=name!='finger_l'
        drive=UsdPhysics.DriveAPI.Apply(j,'angular' if angular else 'linear')
        # USD angular gains are per degree; these are starting simulation gains, not motor tuning.
        drive.CreateStiffnessAttr(30*math.pi/180 if angular else 300)
        drive.CreateDampingAttr(2*math.pi/180 if angular else 10)
        drive.CreateTargetPositionAttr(math.degrees(q) if angular else q)
    tcp=UsdGeom.Xform.Define(stage,'/xsting/gripper/tool0'); tcp.AddTranslateOp().Set(Gf.Vec3d(0.0995,0,0))
    stage.GetRootLayer().Save()
    report={'collision_spheres_per_link':counts,'total_spheres':sum(counts.values()),
            'surface_triangle_coverage':'PASS','planning_dof':5,'fixed_leg_pose':'stand',
            'xrdf_version':1.0,'gpu_planning_test':'not run'}
    (OUT/'generation_report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))

if __name__=='__main__': generate()
