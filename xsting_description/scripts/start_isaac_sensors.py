"""Isaac Sim 6.1 sensor runtime; execute in Script Editor after opening a storage USD.

Use builtins.xsting_sensors.read_camera('front_rgb') / read_ultrasonic().
Camera render products are allocated on demand; Play advances sensor samples.
Executing this script does not change the stage saved on disk.
"""
import builtins,json,math
import numpy as np
import omni.usd
from isaacsim.sensors.experimental.rtx import RtxCamera,CameraSensor
from isaacsim.sensors.experimental.physics import Raycast,RaycastSensor


class XStingSensors:
    def __init__(self):
        self.stage=omni.usd.get_context().get_stage()
        self.stage.SetEditTarget(self.stage.GetSessionLayer())
        self.config={}
        self.cameras={}
        self.range_sensor=None
        self.last_range=None
        self.lidar_sensor=None
        self.last_lidar=None
        for p in self.stage.Traverse():
            raw=p.GetCustomDataByKey('xsting:sensorConfig')
            if not raw:continue
            c=json.loads(raw);c['prim_path']=str(p.GetPath());self.config[c['name']]=c
            if c['name']=='ultrasonic_front':
                self.range_sensor=RaycastSensor(Raycast(c['prim_path'],reset_xform_op_properties=False))
            elif c['name']=='lidar_2d':
                self.lidar_sensor=RaycastSensor(Raycast(c['prim_path'],reset_xform_op_properties=False))
        if not self.config:raise RuntimeError('Open xsting_storage_training/manipulation or inspect_*.usda first')

    def enable_camera(self,name):
        if name not in self.cameras:
            c=self.config[name]
            self.cameras[name]=CameraSensor(RtxCamera(c['prim_path'],reset_xform_op_properties=False),
                resolution=(c['height'],c['width']),annotators=c['annotators'])
        return self.cameras[name]

    def read_camera(self,name):
        """Return a NumPy HxWxC image (RGB uint8 or depth metres), or None during warm-up."""
        data,_=self.enable_camera(name).get_data(self.config[name]['annotators'][0])
        return None if data is None else data.numpy()

    def disable_camera(self,name):
        sensor=self.cameras.pop(name,None)
        if sensor is not None:
            # Isaac Sim 6.1 runtime owns render products but exposes no public close().
            sensor._invalidate_sensor()

    def read_ultrasonic(self):
        """15 Hz nearest cone return; range=+inf means no hit, None means not initialized.

        This is a geometric approximation of ultrasound, without wave propagation.
        Raw rays are exposed for diagnostics; output uses simulation time.
        """
        if self.range_sensor is None:return None
        data=self.range_sensor.get_data()
        depths=np.asarray(data['depths'])
        if not depths.size:return None
        c=self.config['ultrasonic_front'];now=float(data['time'])
        if self.last_range and 0 <= now-self.last_range['time'] < 1/c['rate_hz']:
            return self.last_range
        # Depth == max range is the native no-hit sentinel; reject out-of-range rays.
        valid=np.isfinite(depths)&(depths>=c['min_range_m'])&(depths<c['max_range_m'])
        self.last_range={'time':now,'range_m':float(depths[valid].min()) if valid.any() else math.inf,
            'ray_depths_m':depths.copy(),'hit_prim_paths':list(data['hit_prim_paths'])}
        return self.last_range

    def read_lidar(self, filter_self=True):
        """10 Hz 360-deg 2D scan; out-of-range and self-leg hits are set to +inf."""
        if self.lidar_sensor is None:return None
        data=self.lidar_sensor.get_data()
        depths=np.asarray(data['depths'],dtype=np.float32)
        if not depths.size:return None
        c=self.config['lidar_2d'];now=float(data['time'])
        if self.last_lidar and 0 <= now-self.last_lidar['time'] < 1/c['rate_hz']:
            return self.last_lidar
        ranges=depths.copy()
        invalid=(~np.isfinite(ranges))|(ranges<c['min_range_m'])|(ranges>=c['max_range_m'])
        hits=list(data['hit_prim_paths'])
        if filter_self:
            for i,path in enumerate(hits):
                if path and '/xsting/' in str(path):
                    invalid[i]=True
        ranges[invalid]=math.inf
        self.last_lidar={'time':now,'ranges_m':ranges,'angle_min':0.0,
            'angle_increment':2*math.pi/len(ranges),'hit_prim_paths':hits}
        return self.last_lidar

    def close(self):
        for name in list(self.cameras):self.disable_camera(name)
        self.range_sensor=None
        self.lidar_sensor=None


if __name__=='__main__':
    previous=getattr(builtins,'xsting_sensors',None)
    if previous:previous.close()
    builtins.xsting_sensors=XStingSensors()
    builtins.xsting_sensors.enable_camera('front_rgb')
    print('XSTING_SENSORS_READY',list(builtins.xsting_sensors.config))
