#!/usr/bin/env python3
"""Validate coupled URDF/SRDF/XRDF/USD planning assets without a GPU."""
import itertools
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.distance import cdist
from scipy.spatial.transform import Rotation
import yaml
from pxr import Usd, UsdGeom, UsdPhysics
from generate_assets import ROOT, OUT, ARM, origin

r=ET.parse(OUT/'xsting_manipulation.urdf').getroot()
x=yaml.safe_load((OUT/'xsting.xrdf').read_text())
srdf=ET.parse(OUT/'xsting.srdf').getroot()
links={l.get('name') for l in r.findall('link')}
joints={j.get('name'):j for j in r.findall('joint')}
assert x['cspace']['joint_names']==ARM
assert all(joints[n].get('type')=='fixed' for n in joints if n.startswith('leg'))
assert all(n in links for n in x['tool_frames'])
assert joints['finger_r'].find('mimic').get('joint')=='finger_l'
ignore={tuple(sorted((n,m))) for n,ms in x['self_collision']['ignore'].items() for m in ms}
assert ignore=={tuple(sorted((p.get('link1'),p.get('link2')))) for p in srdf.findall('disable_collisions')}
for group in srdf.findall('group'):
    for j in group.findall('joint'): assert j.get('name') in joints
q=x['default_joint_positions']
tf={'base':np.eye(4)}
pending=list(joints.values())
while pending:
    previous=len(pending)
    for j in pending[:]:
        p=j.find('parent').get('link'); c=j.find('child').get('link')
        if p not in tf: continue
        t=origin(j)
        if j.get('type')=='revolute': t[:3,:3]=t[:3,:3]@Rotation.from_rotvec(np.fromstring(j.find('axis').get('xyz'),sep=' ')*q[j.get('name')]).as_matrix()
        if j.get('type')=='prismatic': t[:3,3]+=np.fromstring(j.find('axis').get('xyz'),sep=' ')*q[j.get('name')]
        tf[c]=tf[p]@t;pending.remove(j)
    assert len(pending)<previous
spheres=x['geometry']['robot_spheres']['spheres']
positions={n:np.array([v['center'] for v in values])@tf[n][:3,:3].T+tf[n][:3,3] for n,values in spheres.items()}
collisions=[]
for a,b in itertools.combinations(spheres,2):
    if tuple(sorted((a,b))) in ignore:continue
    d=cdist(positions[a],positions[b])-np.array([v['radius'] for v in spheres[a]])[:,None]-np.array([v['radius'] for v in spheres[b]])[None,:]
    if d.min()<0:collisions.append((a,b,float(d.min())))
assert not collisions,collisions
stage=Usd.Stage.Open(str(ROOT/'xsting_description/usd/xsting_manipulation.usda'))
roots=[p for p in stage.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
assert len(roots)==1 and roots[0].GetName()=='world_anchor'
cache=UsdGeom.XformCache()
base=np.array(cache.GetLocalToWorldTransform(stage.GetPrimAtPath('/xsting/base'))).T
for p in stage.Traverse():
    if p.HasAPI(UsdPhysics.RigidBodyAPI):
        actual=np.array(cache.GetLocalToWorldTransform(p)).T
        assert np.allclose(actual,base@tf[p.GetName()],atol=1e-6),p.GetName()
for n in joints:
    if n.startswith('leg'):
        j=stage.GetPrimAtPath('/xsting/Physics/'+n)
        assert j.GetAttribute('physics:lowerLimit').Get()==j.GetAttribute('physics:upperLimit').Get()
controllers=yaml.safe_load((OUT/'ros2_controllers.yaml').read_text())
assert controllers['arm_trajectory_controller']['ros__parameters']['joints']==ARM
assert controllers['gripper_trajectory_controller']['ros__parameters']['joints']==['finger_l']
report={'asset_consistency':'PASS','initial_sphere_self_collision':'NONE','urdf_usd_forward_kinematics':'MATCH',
        'arm_dof':5,'sphere_count':sum(map(len,spheres.values())),
        'physx_cumotion_execution':'NOT TESTED: GPU unavailable'}
(OUT/'validation.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
