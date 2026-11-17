import asyncio,builtins,json,sys
from pathlib import Path
import omni.usd,omni.ui as ui,carb
from omni.kit.viewport.utility import get_active_viewport
from omni.physx.bindings import _physx
from pxr import UsdGeom,Gf
folder=Path(omni.usd.get_context().get_stage_url()).parent
async def show_kind(kind):
    runtime=getattr(builtins,'xsting_sensors',None)
    if runtime:runtime.close()
    ctx=omni.usd.get_context()
    await ctx.open_stage_async(str(folder/f'inspect_{kind}.usda'))
    stage=ctx.get_stage();stage.SetEditTarget(stage.GetSessionLayer())
    for r in json.loads((folder/'runtime_validation.json').read_text())['results']:
        if r['shape']==kind:
            ops=UsdGeom.Xformable(stage.GetPrimAtPath('/World/Pucks/'+r['slot'])).GetOrderedXformOps()
            ops[0].Set(Gf.Vec3d(*r['position']))
            q=r['quaternion_wxyz'];ops[1].Set(Gf.Quatf(q[0],Gf.Vec3f(*q[1:])))
    get_active_viewport().camera_path='/World/Camera'
    carb.settings.get_settings().set_int(_physx.SETTING_DISPLAY_COLLIDERS,2)
def switch(kind):asyncio.ensure_future(show_kind(kind))
def camera_view(name):
    for p in omni.usd.get_context().get_stage().Traverse():
        if p.GetName()=='Sensor_'+name:
            get_active_viewport().camera_path=str(p.GetPath())
            carb.settings.get_settings().set_int(_physx.SETTING_DISPLAY_COLLIDERS,0)
            return
def start_sensors():
    sys.path.insert(0,str(folder.parent.parent/'scripts'))
    from start_isaac_sensors import XStingSensors
    old=getattr(builtins,'xsting_sensors',None)
    if old:old.close()
    builtins.xsting_sensors=XStingSensors()
    builtins.xsting_sensors.enable_camera('front_rgb')
def select_front_collider():
    omni.usd.get_context().get_selection().set_selected_prim_paths(
        ['/World/xsting/base/StorageColliders/FrontRealSense'],True)
    get_active_viewport().camera_path='/World/Camera'
    carb.settings.get_settings().set_int(_physx.SETTING_DISPLAY_COLLIDERS,2)
w=ui.Window('xSting - Puck inspection',width=370,height=295)
with w.frame:
    with ui.VStack(spacing=6):
        ui.Label('Measured seated poses (24-drop test)')
        with ui.HStack():
            ui.Button('Square 30 x 30 x 60',clicked_fn=lambda:switch('square'))
            ui.Button('Round D30 x 60',clicked_fn=lambda:switch('round'))
        with ui.HStack():
            ui.Button('Colliders ON',clicked_fn=lambda:carb.settings.get_settings().set_int(_physx.SETTING_DISPLAY_COLLIDERS,2))
            ui.Button('Colliders OFF',clicked_fn=lambda:carb.settings.get_settings().set_int(_physx.SETTING_DISPLAY_COLLIDERS,0))
        ui.Label('Front two slots: tilted support (~3.3-3.8 deg)')
        ui.Label('Camera views (RGB viewport)')
        with ui.HStack():
            for label,name in [('Front','front_rgb'),('Left','left_rgb'),('Right','right_rgb'),('Arm','arm_rgb')]:
                ui.Button(label,clicked_fn=lambda n=name:camera_view(n))
        with ui.HStack():
            ui.Button('RealSense collider',clicked_fn=select_front_collider)
            ui.Button('Enable sensors',clicked_fn=start_sensors)
        ui.Label('Play, then read builtins.xsting_sensors in Python')
builtins.xsting_storage_inspector=w
