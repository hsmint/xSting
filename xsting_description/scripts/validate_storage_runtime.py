#!/usr/bin/env python3
"""Compare measured PhysX puck poses to the original basket triangles with FCL."""
from pathlib import Path
import json,hashlib
import fcl
import numpy as np
from scipy.spatial.transform import Rotation
from pxr import Usd, UsdGeom

folder=Path(__file__).resolve().parents[1]/'usd/storage'
stage=Usd.Stage.Open(str(folder/'storage_colliders.usdc'))
m=UsdGeom.Mesh(stage.GetPrimAtPath('/StorageColliders/Basket'))
v=np.array(m.GetPointsAttr().Get(),dtype=np.float64)
f=np.array(m.GetFaceVertexIndicesAttr().Get()).reshape(-1,3)
b=fcl.BVHModel();b.beginModel(len(v),len(f));b.addSubModel(v,f);b.endModel()
basket=fcl.CollisionObject(b)
report=json.loads((folder/'runtime_validation.json').read_text())
assert report['collider_sha256']==hashlib.sha256((folder/'storage_colliders.usdc').read_bytes()).hexdigest(), 'Stale runtime results: rerun PhysX test'
slots={s['name']:s for s in json.loads((folder/'slots.json').read_text())}
for p in report['results']:
    q=p['quaternion_wxyz'];rot=Rotation.from_quat(q[1:]+q[:1]).as_matrix()
    xyz=np.array(p['position']);xyz[2]-=.04
    shape=fcl.Box(.03,.03,.06) if p['shape']=='square' else fcl.Cylinder(.015,.06)
    puck=fcl.CollisionObject(shape,fcl.Transform(rot,xyz));contacts=fcl.CollisionResult()
    fcl.collide(basket,puck,fcl.CollisionRequest(num_max_contacts=10000,enable_contact=True),contacts)
    p['source_mesh_max_contact_depth_mm']=max([c.penetration_depth for c in contacts.contacts]+[0])*1000
    p['tilt_degrees']=float(np.degrees(np.arccos(np.clip(rot[2,2],-1,1))))
    p['xy_drift_mm']=float(np.linalg.norm(xyz[:2]-np.array(slots[p['slot']]['xyz'][:2]))*1000)
report['max_source_mesh_contact_depth_mm']=max(p['source_mesh_max_contact_depth_mm'] for p in report['results'])
report['all_pucks_retained']=all(abs(p['center_height_error_mm'])<1 and p['xy_drift_mm']<3 for p in report['results'])
assert len(report['results'])==24
assert report['all_pucks_retained']
assert report['max_source_mesh_contact_depth_mm']<.1
report['validation']='PASS: 24 drops, retained; source mesh contact depth <0.1 mm'
report['limitations']='3 seconds at 240 Hz, fixed base, 5 mm vertical release; not a general error bound or shaking/insertion policy validation'
(folder/'runtime_validation.json').write_text(json.dumps(report,indent=2)+'\n')
print(report['validation']);print('max contact depth mm:',report['max_source_mesh_contact_depth_mm'])
