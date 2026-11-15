#!/usr/bin/env python3
"""Author camera and ultrasonic-range sensor prims at the existing URDF frames."""
from pathlib import Path
import math,json
import yaml
from pxr import UsdGeom,Sdf,Gf,Vt
PACKAGE=Path(__file__).resolve().parents[1]

def attribute(p,n,t,v):p.CreateAttribute(n,t,custom=False).Set(v)
def add_sensors(stage):
    frames={p.GetName():p.GetPath() for p in stage.Traverse()}
    configs=[]
    for name,frame in [('front','camera'),('left','left_camera'),('right','right_camera')]:
        for mode,hfov in [('rgb',69),('depth',87)]:
            optical=frame+('_color_frame' if mode=='rgb' else '_depth_frame')
            f=320/math.tan(math.radians(hfov/2))
            configs.append({'name':name+'_'+mode,'frame':optical,'width':640,'height':480,
                'fx':f,'fy':f,'cx':320.0,'cy':240.0,'rate_hz':30.0,
                'annotators':['rgb'] if mode=='rgb' else ['distance_to_image_plane'],
                'calibration':'nominal horizontal FOV; square pixels; not device calibration'})
    arm=yaml.safe_load((PACKAGE.parent/'xsting_vision/config/arducam_camera.yaml').read_text())['/**']['ros__parameters']
    P=arm['projection_matrix']
    configs.append({'name':'arm_rgb','frame':'arducam_frame','width':640,'height':360,
        'fx':P[0]/3,'fy':P[5]/3,'cx':P[2]/3,'cy':P[6]/3,'rate_hz':30.0,'annotators':['rgb'],
        'calibration':'rectified projection_matrix from arducam_camera.yaml scaled from 1920x1080 to 640x360'})
    for c in configs:
        camera=UsdGeom.Camera.Define(stage,frames[c['frame']].AppendChild('Sensor_'+c['name']))
        camera.AddOrientOp().Set(Gf.Quatf(0,1,0,0)) # ROS optical +Z/+Y-down -> USD -Z/+Y-up
        camera.CreateFocalLengthAttr(1.0)
        camera.CreateHorizontalApertureAttr(c['width']/c['fx'])
        camera.CreateVerticalApertureAttr(c['height']/c['fy'])
        camera.CreateHorizontalApertureOffsetAttr((c['width']/2-c['cx'])/c['fx'])
        camera.CreateVerticalApertureOffsetAttr((c['cy']-c['height']/2)/c['fy'])
        camera.CreateClippingRangeAttr(Gf.Vec2f(.01 if c['name']=='arm_rgb' else .05,20))
        camera.CreateFStopAttr(0)
        p=camera.GetPrim();p.AddAppliedSchema('OmniSensorAPI')
        p.AddAppliedSchema('OmniLensDistortionOpenCvPinholeAPI')
        attribute(p,'omni:sensor:tickRate',Sdf.ValueTypeNames.Float,c['rate_hz'])
        for key in ['fx','fy','cx','cy']:
            attribute(p,'omni:lensdistortion:opencvPinhole:'+key,Sdf.ValueTypeNames.Float,c[key])
        attribute(p,'omni:lensdistortion:opencvPinhole:imageSize',Sdf.ValueTypeNames.Int2,Gf.Vec2i(c['width'],c['height']))
        for key in ['k1','k2','p1','p2','k3','k4','k5','k6']:
            attribute(p,'omni:lensdistortion:opencvPinhole:'+key,Sdf.ValueTypeNames.Float,0.0)
        p.SetCustomDataByKey('xsting:sensorConfig',json.dumps(c))
        c['prim_path']=str(p.GetPath())
    directions=[(1.,0.,0.)]
    for deg,count in [(3.75,8),(7.5,16)]:
        theta=math.radians(deg)
        for i in range(count):
            a=2*math.pi*i/count;directions.append((math.cos(theta),math.sin(theta)*math.cos(a),math.sin(theta)*math.sin(a)))
    p=stage.DefinePrim(frames['ultrasonic'].AppendChild('RangeSensor'),'IsaacRaycastSensor')
    for n,t,v in [('enabled',Sdf.ValueTypeNames.Bool,True),('minRange',Sdf.ValueTypeNames.Float,.02),
        ('maxRange',Sdf.ValueTypeNames.Float,4.),('numRays',Sdf.ValueTypeNames.UInt,len(directions)),
        ('outputFrameOfReference',Sdf.ValueTypeNames.Token,'SENSOR'),('reportHitPrimPaths',Sdf.ValueTypeNames.Bool,True),
        ('rayDirections',Sdf.ValueTypeNames.Float3Array,Vt.Vec3fArray(directions)),
        ('rayOrigins',Sdf.ValueTypeNames.Float3Array,Vt.Vec3fArray([(0,0,0)]*len(directions))),
        ('rayTimeOffsets',Sdf.ValueTypeNames.FloatArray,Vt.FloatArray([0]*len(directions)))]:attribute(p,n,t,v)
    ultrasound={'name':'ultrasonic_front','frame':'ultrasonic','prim_path':str(p.GetPath()),
        'min_range_m':.02,'max_range_m':4.0,'fov_degrees':15,'rate_hz':15.0,'rays':25,
        'model':'geometric nearest return in cone; no acoustic waveform, multipath or material response'}
    p.SetCustomDataByKey('xsting:sensorConfig',json.dumps(ultrasound))
    lidar_dirs=[(math.cos(2*math.pi*i/720),math.sin(2*math.pi*i/720),0.) for i in range(720)]
    lp=stage.DefinePrim(frames['lidar'].AppendChild('LidarSensor'),'IsaacRaycastSensor')
    for n,t,v in [('enabled',Sdf.ValueTypeNames.Bool,True),('minRange',Sdf.ValueTypeNames.Float,.145),
        ('maxRange',Sdf.ValueTypeNames.Float,12.),('numRays',Sdf.ValueTypeNames.UInt,len(lidar_dirs)),
        ('outputFrameOfReference',Sdf.ValueTypeNames.Token,'SENSOR'),('reportHitPrimPaths',Sdf.ValueTypeNames.Bool,True),
        ('rayDirections',Sdf.ValueTypeNames.Float3Array,Vt.Vec3fArray(lidar_dirs)),
        ('rayOrigins',Sdf.ValueTypeNames.Float3Array,Vt.Vec3fArray([(0,0,0)]*len(lidar_dirs))),
        ('rayTimeOffsets',Sdf.ValueTypeNames.FloatArray,Vt.FloatArray([0]*len(lidar_dirs)))]:attribute(lp,n,t,v)
    lidar={'name':'lidar_2d','frame':'lidar','prim_path':str(lp.GetPath()),
        'min_range_m':.145,'max_range_m':12.0,'fov_degrees':360,'rate_hz':10.0,'rays':720,
        'model':'360 deg planar 2D ToF raycast (COIN-D4 / LDS-03); min_range clears chassis standoffs'}
    lp.SetCustomDataByKey('xsting:sensorConfig',json.dumps(lidar))
    (PACKAGE/'config/isaac_sensors.json').write_text(json.dumps({'cameras':configs,'ultrasonic':ultrasound,'lidar':lidar},indent=2)+'\n')
