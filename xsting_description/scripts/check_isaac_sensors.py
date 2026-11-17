"""Execute in Isaac Sim Script Editor on inspect_*.usda (replaces session edits).
Checks 7 camera outputs, a 0.5 m ultrasound target and front housing collisions.
Temporary fixtures and sensor render products are removed after validation.
"""
import asyncio,builtins,json,sys,traceback,hashlib
from pathlib import Path
import numpy as np
import omni.usd,omni.kit.app,omni.timeline,omni.physx,carb
from pxr import UsdGeom,UsdPhysics,Gf
from omni.physx.bindings import _physx


async def check_sensors():
    ctx=omni.usd.get_context();app=omni.kit.app.get_app();timeline=omni.timeline.get_timeline_interface()
    folder=Path(ctx.get_stage_url()).parent
    package=folder.parent.parent
    (folder/'sensor_validation.json').unlink(missing_ok=True)
    (folder/'sensor_error.txt').unlink(missing_ok=True)
    sys.path.insert(0,str(package/'scripts'))
    from start_isaac_sensors import XStingSensors
    old=getattr(builtins,'xsting_sensors',None)
    if old:old.close()
    s=None
    try:
        timeline.stop()
        stage=ctx.get_stage();stage.SetEditTarget(stage.GetSessionLayer())
        carb.settings.get_settings().set_int(_physx.SETTING_DISPLAY_COLLIDERS,0)
        s=XStingSensors();builtins.xsting_sensors=s
        timeline.play()
        for _ in range(60):await app.next_update_async()
        report={'isaac_sim':'6.1','cameras':[],
            'collider_sha256':hashlib.sha256((folder/'storage_colliders.usdc').read_bytes()).hexdigest(),
            'sensor_config_sha256':hashlib.sha256((package/'config/isaac_sensors.json').read_bytes()).hexdigest()}
        for name,c in s.config.items():
            if 'annotators' not in c:continue
            tf=UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(c['prim_path']))
            # Flat, visible target with its front face 0.5 m along the optical axis.
            target=UsdGeom.Cube.Define(stage,'/World/SensorTestTarget')
            target.CreateSizeAttr(1)
            target.AddTransformOp().Set(Gf.Matrix4d().SetScale(Gf.Vec3d(.4,.4,.01))*
                Gf.Matrix4d().SetTranslate(Gf.Vec3d(0,0,-.505))*tf)
            target.CreateDisplayColorAttr([Gf.Vec3f(.8,.08,.04)])
            s.enable_camera(name)
            for _ in range(90):await app.next_update_async()
            data=s.read_camera(name)
            assert data is not None,name+' returned no data'
            assert data.shape[:2]==(c['height'],c['width']),data.shape
            r={'name':name,'shape':list(data.shape),'dtype':str(data.dtype)}
            if 'depth' in name:
                x=int(round(c['cx']));y=int(round(c['cy']))
                depth=float(np.median(data[y-2:y+3,x-2:x+3]))
                r['target_depth_m']=depth
                assert abs(depth-.5)<.01,(name,depth)
            else:
                r['pixel_std']=float(data.std())
                assert r['pixel_std']>5,(name,'constant/blank output')
                from PIL import Image
                Image.fromarray(data).save('/tmp/xsting_'+name+'.png')
            report['cameras'].append(r)
            print('SENSOR_CAMERA_PASS',r,flush=True)
            s.disable_camera(name);stage.RemovePrim('/World/SensorTestTarget')
            for _ in range(5):await app.next_update_async()
        u=s.config['ultrasonic_front']
        tf=UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(u['prim_path']))
        target=UsdGeom.Cube.Define(stage,'/World/UltrasonicTestTarget');target.CreateSizeAttr(1)
        target.AddTransformOp().Set(Gf.Matrix4d().SetScale(Gf.Vec3d(.02,.25,.25))*
            Gf.Matrix4d().SetTranslate(Gf.Vec3d(.51,0,0))*tf)
        UsdPhysics.CollisionAPI.Apply(target.GetPrim())
        for _ in range(30):await app.next_update_async()
        s.last_range=None;r=s.read_ultrasonic()
        assert r and abs(r['range_m']-.5)<.001,r
        report['ultrasonic']={'expected_m':.5,'measured_m':r['range_m'],
            'rays':len(r['ray_depths_m']),'hit_rays':sum(bool(p) for p in r['hit_prim_paths']),
            'model':u['model']}
        stage.RemovePrim('/World/UltrasonicTestTarget')
        for _ in range(10):await app.next_update_async()
        s.last_range=None;r=s.read_ultrasonic()
        assert r and np.isinf(r['range_m']),r
        report['ultrasonic']['no_target_range']='inf'
        # Verify cooked camera hull is queryable from all six sides, in world space.
        expected=json.loads((folder/'geometry_validation.json').read_text())['front_realsense']
        low=np.array(expected['min_m']);high=np.array(expected['max_m']);center=(low+high)/2
        base=UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath('/World/xsting/base'))
        query=omni.physx.get_physx_scene_query_interface();hits=[]
        for axis in range(3):
            for side in [-1,1]:
                # Begin 3 mm outside the camera envelope so a rear-facing probe
                # does not start inside the chassis floor slab behind the camera.
                origin=center.copy();origin[axis]=(high if side>0 else low)[axis]+side*.003
                direction=np.zeros(3);direction[axis]=-side
                hit=query.raycast_closest(tuple(base.Transform(Gf.Vec3d(*origin))),
                    tuple(base.TransformDir(Gf.Vec3d(*direction))),.2)
                assert hit['hit'] and 'FrontRealSense' in hit['collision'],hit
                position=base.GetInverse().Transform(Gf.Vec3d(*hit['position']))
                # Rounded side profiles can be inside the global source AABB.
                coordinate=float(position[axis]);bound=(high if side>0 else low)[axis]
                assert abs(coordinate-bound)<.002,(axis,side,coordinate,bound)
                hits.append({'axis':axis,'side':side,'collision':hit['collision'],
                    'position_base_m':list(position),'aabb_delta_mm':abs(coordinate-bound)*1000})
        report['front_realsense_rays']=hits;report['result']='PASS'
        (folder/'sensor_validation.json').write_text(json.dumps(report,indent=2)+'\n')
        print('SENSOR_TESTS_COMPLETE',flush=True)
    except Exception:
        (folder/'sensor_error.txt').write_text(traceback.format_exc());print(traceback.format_exc())
    finally:
        timeline.pause()
        if s:s.close()
        stage=ctx.get_stage()
        for path in ['/World/SensorTestTarget','/World/UltrasonicTestTarget']:
            if stage.GetPrimAtPath(path):stage.RemovePrim(path)
        carb.settings.get_settings().set_int(_physx.SETTING_DISPLAY_COLLIDERS,2)

asyncio.ensure_future(check_sensors())
