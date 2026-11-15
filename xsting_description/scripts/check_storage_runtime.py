"""Run inside Isaac Sim after opening a storage asset; replaces the current stage.
Produces raw runtime_validation.json; assess it with validate_storage_runtime.py.
"""
import asyncio,json,math,traceback,hashlib,builtins
from pathlib import Path
import omni.usd,omni.kit.app,omni.physx,omni.timeline,carb
from omni.kit.viewport.utility import get_active_viewport
from omni.physx.bindings import _physx
from pxr import UsdGeom,Usd,Sdf

async def check_storage():
    directory=Path(omni.usd.get_context().get_stage_url()).parent
    if directory.name != 'storage': directory /= 'storage'
    reference=json.loads((directory/'geometry_validation.json').read_text())
    (directory/'runtime_error.txt').unlink(missing_ok=True)
    runtime=getattr(builtins,'xsting_sensors',None)
    if runtime:runtime.close()
    omni.timeline.get_timeline_interface().stop()
    results=[]
    ctx=omni.usd.get_context();app=omni.kit.app.get_app()
    physx=omni.physx.get_physx_interface();simulation=omni.physx.get_physx_simulation_interface()
    try:
        for kind in ['square','round']:
            await ctx.open_stage_async(str(directory/f'inspect_{kind}.usda'))
            stage=ctx.get_stage();stage.SetEditTarget(stage.GetSessionLayer())
            get_active_viewport().camera_path='/World/Camera'
            carb.settings.get_settings().set_int(_physx.SETTING_DISPLAY_COLLIDERS,0)
            for _ in range(15):await app.next_update_async()
            print('STORAGE_START',kind,flush=True)
            physx.start_simulation()
            for i in range(720):
                simulation.simulate(1/240,i/240)
                simulation.fetch_results()
                physx.update_transformations(False,True)
                if i%16==0:await app.next_update_async()
            cache=UsdGeom.XformCache()
            for expected in reference['support_tests']:
                if expected['shape']!=kind:continue
                prim=stage.GetPrimAtPath('/World/Pucks/'+expected['slot'])
                tf=cache.GetLocalToWorldTransform(prim);pos=tf.ExtractTranslation();rot=tf.ExtractRotationQuat()
                bottom=pos[2]-.03-.04
                results.append({'shape':kind,'slot':expected['slot'],'position':list(pos),
                    'quaternion_wxyz':[rot.GetReal(),*rot.GetImaginary()],
                    'expected_support_z':expected['support_bottom_z'],
                    'center_height_error_mm':(bottom-expected['support_bottom_z'])*1000})
            print('STORAGE_DONE',kind,flush=True)
            physx.reset_simulation()
        (directory/'runtime_validation.json').write_text(json.dumps({'steps':720,'dt':1/240,'collider_sha256':hashlib.sha256((directory/'storage_colliders.usdc').read_bytes()).hexdigest(),'results':results},indent=2)+'\n')
        await ctx.open_stage_async(str(directory/'inspect_square.usda'))
        stage=ctx.get_stage();stage.SetEditTarget(stage.GetSessionLayer())
        # Show the measured seated positions for inspection; don't save over source test spawn poses.
        for r in results:
            if r['shape']=='square':
                from pxr import Gf
                p=UsdGeom.Xformable(stage.GetPrimAtPath('/World/Pucks/'+r['slot']))
                ops=p.GetOrderedXformOps();ops[0].Set(Gf.Vec3d(*r['position']));ops[1].Set(Gf.Quatf(r['quaternion_wxyz'][0],Gf.Vec3f(*r['quaternion_wxyz'][1:])))
        get_active_viewport().camera_path='/World/Camera'
        carb.settings.get_settings().set_int(_physx.SETTING_DISPLAY_COLLIDERS,2)
        print('STORAGE_TESTS_COMPLETE',flush=True)
    except Exception:
        (directory/'runtime_error.txt').write_text(traceback.format_exc());print(traceback.format_exc())
asyncio.ensure_future(check_storage())
