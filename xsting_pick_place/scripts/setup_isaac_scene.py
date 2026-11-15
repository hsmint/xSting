#!/usr/bin/env python3
"""Isaac Sim 6용 xSting 상자·퍽 물리 스테이지 및 ROS 2 브리지 구성."""

import math
import omni.graph.core as og
import omni.kit.app
import omni.timeline
import omni.usd
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade

USD_PATH = (
    '/home/pa03/Documents/FINAL/mk2_ws/src/xsting_description/usd/'
    'xsting_storage_manipulation.usda'
)
ROBOT = '/xsting'
GRAPH = '/xsting_ros2'

# pick_place.yaml 및 USD 발끝 실측치(bz - 0.02133m)와 일치하는 상수
GROUND_OFF_Z = -0.0213
BOX_X = 0.285
BOX_Y = 0.0


# 마찰 계수가 높은 그리퍼·퍽 공용 물리 재질 생성
def make_grip_mat(stage, path):
    mat = UsdShade.Material.Define(stage, path)
    phys = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    phys.CreateStaticFrictionAttr(1.5)
    phys.CreateDynamicFrictionAttr(1.2)
    phys.CreateRestitutionAttr(0.0)
    return mat


# 고정 충돌체(지면·작업 상자) 생성
def add_static_box(stage, path, pos, size, color, mat):
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    xf = UsdGeom.XformCommonAPI(cube)
    xf.SetTranslate(Gf.Vec3d(*pos))
    xf.SetScale(Gf.Vec3f(*size))
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(
        mat, UsdShade.Tokens.weakerThanDescendants, 'physics'
    )


# 32각 볼록 원기둥 및 사각 강체 퍽 생성(SDF 정밀 접촉용 0.2mm 오프셋 적용)
def add_puck(stage, path, is_round, pos, rot_deg, color, mat):
    body = UsdGeom.Xform.Define(stage, path)
    xf = UsdGeom.XformCommonAPI(body)
    xf.SetTranslate(Gf.Vec3d(*pos))
    xf.SetRotate(Gf.Vec3f(*rot_deg), UsdGeom.XformCommonAPI.RotationOrderXYZ)

    body_prim = body.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(body_prim)
    UsdPhysics.MassAPI.Apply(body_prim).CreateMassAttr(
        0.0424 if is_round else 0.054
    )
    rb = PhysxSchema.PhysxRigidBodyAPI.Apply(body_prim)
    rb.CreateSolverPositionIterationCountAttr(32)
    rb.CreateSolverVelocityIterationCountAttr(4)

    geom_path = f'{path}/Geom'
    if is_round:
        mesh = UsdGeom.Mesh.Define(stage, geom_path)
        pts = [Gf.Vec3f(0.0, 0.0, -0.030), Gf.Vec3f(0.0, 0.0, 0.030)]
        idx = []
        seg = 32
        for i in range(seg):
            ang = 2.0 * math.pi * i / seg
            cx, sy = 0.015 * math.cos(ang), 0.015 * math.sin(ang)
            pts.append(Gf.Vec3f(cx, sy, -0.030))
            pts.append(Gf.Vec3f(cx, sy, 0.030))
            j = (i + 1) % seg
            b0, t0 = 2 + 2 * i, 3 + 2 * i
            b1, t1 = 2 + 2 * j, 3 + 2 * j
            idx.extend([0, b1, b0, 1, t0, t1, b0, b1, t1, b0, t1, t0])
        mesh.CreatePointsAttr(pts)
        mesh.CreateFaceVertexCountsAttr([3] * (seg * 4))
        mesh.CreateFaceVertexIndicesAttr(idx)
        mesh.CreateExtentAttr(
            [Gf.Vec3f(-0.015, -0.015, -0.030), Gf.Vec3f(0.015, 0.015, 0.030)]
        )
        mesh.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        gprim = mesh.GetPrim()
        UsdPhysics.CollisionAPI.Apply(gprim)
        UsdPhysics.MeshCollisionAPI.Apply(gprim).CreateApproximationAttr(
            'convexHull'
        )
    else:
        cube = UsdGeom.Cube.Define(stage, geom_path)
        cube.CreateSizeAttr(1.0)
        UsdGeom.XformCommonAPI(cube).SetScale(Gf.Vec3f(0.030, 0.030, 0.060))
        cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        gprim = cube.GetPrim()
        UsdPhysics.CollisionAPI.Apply(gprim)

    pcol = PhysxSchema.PhysxCollisionAPI.Apply(gprim)
    pcol.CreateContactOffsetAttr(0.0002)
    pcol.CreateRestOffsetAttr(0.0)
    UsdShade.MaterialBindingAPI.Apply(gprim).Bind(
        mat, UsdShade.Tokens.weakerThanDescendants, 'physics'
    )


# ArticulationRootAPI 정규화 및 팔·그리퍼 서보 강성·마찰 보강
def tune_robot_physics(stage, grip_mat):
    anchor = stage.GetPrimAtPath(ROBOT + '/Physics/world_anchor')
    if anchor and anchor.HasAPI(UsdPhysics.ArticulationRootAPI):
        anchor.RemoveAPI(UsdPhysics.ArticulationRootAPI)
        anchor.RemoveAppliedSchema('PhysxArticulationAPI')

    top = stage.GetPrimAtPath(ROBOT)
    if top:
        if not top.HasAPI(UsdPhysics.ArticulationRootAPI):
            UsdPhysics.ArticulationRootAPI.Apply(top)
            top.AddAppliedSchema('PhysxArticulationAPI')
        top.CreateAttribute(
            'physxArticulation:enabledSelfCollisions', Sdf.ValueTypeNames.Bool
        ).Set(False)
        top.CreateAttribute(
            'physxArticulation:solverPositionIterationCount',
            Sdf.ValueTypeNames.Int,
        ).Set(32)
        top.CreateAttribute(
            'physxArticulation:solverVelocityIterationCount',
            Sdf.ValueTypeNames.Int,
        ).Set(4)

    # 실기 하드웨어 접힌 시작 자세(kArmHome) 동기화 및 안정 서보 게인 설정
    home_deg = {
        'arm_yaw': 0.0,
        'arm_shoulder': math.degrees(-1.0508),
        'arm_elbow': math.degrees(1.0707),
        'arm_wrist': 0.0,
        'arm_roll': 0.0,
    }
    for j_name, deg in home_deg.items():
        jp = stage.GetPrimAtPath(f'{ROBOT}/Physics/{j_name}')
        if not jp:
            continue
        jp.CreateAttribute(
            'drive:angular:physics:stiffness', Sdf.ValueTypeNames.Float
        ).Set(6.0)
        jp.CreateAttribute(
            'drive:angular:physics:damping', Sdf.ValueTypeNames.Float
        ).Set(0.12)
        jp.CreateAttribute(
            'drive:angular:physics:maxForce', Sdf.ValueTypeNames.Float
        ).Set(25.0)
        jp.CreateAttribute(
            'drive:angular:physics:targetPosition', Sdf.ValueTypeNames.Float
        ).Set(deg)
        jp.CreateAttribute(
            'state:angular:physics:position', Sdf.ValueTypeNames.Float
        ).Set(deg)

    # 양쪽 손가락 대칭 리니어 서보 구동 및 파지내 회전 순응 게인 설정
    for f_name in ('finger_l', 'finger_r'):
        fp = stage.GetPrimAtPath(f'{ROBOT}/Physics/{f_name}')
        if not fp:
            continue
        drv = UsdPhysics.DriveAPI.Apply(fp, 'linear')
        drv.CreateTypeAttr('force')
        drv.CreateStiffnessAttr(2000.0)
        drv.CreateDampingAttr(70.0)
        drv.CreateMaxForceAttr(20.0)
        fp.CreateAttribute(
            'physxJoint:maxJointVelocity', Sdf.ValueTypeNames.Float
        ).Set(0.12)

    # 손가락 끝단 수평 롤러 패드(반경 5mm)로 미세 이완 시 파지내 중력 회전 유도
    pad_specs = (
        ('finger_l', 'finger_left_1', -0.0035),
        ('finger_r', 'finger_right_1', 0.0035),
    )
    for link_name, hull_name, y_off in pad_specs:
        hull = stage.GetPrimAtPath(f'{ROBOT}/{link_name}/{hull_name}')
        if hull:
            hull.SetActive(False)
        pad_path = f'{ROBOT}/{link_name}/PadCol'
        if stage.GetPrimAtPath(pad_path):
            stage.RemovePrim(pad_path)
        pad = UsdGeom.Cylinder.Define(stage, pad_path)
        pad.CreateAxisAttr('Z')
        pad.CreateRadiusAttr(0.0050)
        pad.CreateHeightAttr(0.011)
        pad.CreatePurposeAttr(UsdGeom.Tokens.guide)
        UsdGeom.XformCommonAPI(pad).SetTranslate(Gf.Vec3d(0.030, y_off, 0.0))
        UsdPhysics.CollisionAPI.Apply(pad.GetPrim())
        pcol = PhysxSchema.PhysxCollisionAPI.Apply(pad.GetPrim())
        pcol.CreateTorsionalPatchRadiusAttr(0.0020)
        pcol.CreateContactOffsetAttr(0.0002)
        pcol.CreateRestOffsetAttr(0.0)
        UsdShade.MaterialBindingAPI.Apply(pad.GetPrim()).Bind(
            grip_mat, UsdShade.Tokens.strongerThanDescendants, 'physics'
        )


# Isaac Sim 관절 상태·명령·시계 및 팔 ArduCam ROS 2 OmniGraph 구성
def build_ros2_graph(stage):
    if stage.GetPrimAtPath(GRAPH):
        stage.RemovePrim(GRAPH)
    keys = og.Controller.Keys
    nodes = [
        ('Tick', 'omni.graph.action.OnPlaybackTick'),
        ('Time', 'isaacsim.core.nodes.IsaacReadSimulationTime'),
        ('Read', 'isaacsim.sensors.physics.IsaacReadJointState'),
        ('Context', 'isaacsim.ros2.bridge.ROS2Context'),
        ('Publish', 'isaacsim.ros2.bridge.ROS2PublishJointState'),
        ('Subscribe', 'isaacsim.ros2.bridge.ROS2SubscribeJointState'),
        ('Control', 'isaacsim.core.nodes.IsaacArticulationController'),
        ('Clock', 'isaacsim.ros2.bridge.ROS2PublishClock'),
    ]
    conns = [
        ('Tick.outputs:tick', 'Read.inputs:execIn'),
        ('Read.outputs:execOut', 'Publish.inputs:execIn'),
        ('Tick.outputs:tick', 'Subscribe.inputs:execIn'),
        ('Subscribe.outputs:execOut', 'Control.inputs:execIn'),
        ('Tick.outputs:tick', 'Clock.inputs:execIn'),
        ('Time.outputs:simulationTime', 'Publish.inputs:timeStamp'),
        ('Time.outputs:simulationTime', 'Clock.inputs:timeStamp'),
        ('Subscribe.outputs:jointNames', 'Control.inputs:jointNames'),
        ('Subscribe.outputs:positionCommand', 'Control.inputs:positionCommand'),
    ]
    vals = [
        ('Context.inputs:useDomainIDEnvVar', True),
        ('Control.inputs:robotPath', ROBOT),
        ('Read.inputs:prim', [Sdf.Path(ROBOT)]),
        ('Publish.inputs:topicName', '/isaac_joint_states'),
        ('Subscribe.inputs:topicName', '/isaac_joint_commands'),
        ('Clock.inputs:topicName', '/clock'),
    ]
    for field in (
        'jointNames', 'jointPositions', 'jointVelocities',
        'jointEfforts', 'jointDofTypes', 'stageMetersPerUnit', 'sensorTime',
    ):
        conns.append((f'Read.outputs:{field}', f'Publish.inputs:{field}'))
    for name in ('Publish', 'Subscribe', 'Clock'):
        conns.append(('Context.outputs:context', f'{name}.inputs:context'))

    cam_path = f'{ROBOT}/gripper/arducam/arducam_frame/Sensor_arm_rgb'
    cam_prim = stage.GetPrimAtPath(cam_path)
    if not cam_prim:
        for pr in Usd.PrimRange(stage.GetPrimAtPath(ROBOT)):
            if pr.GetName() == 'Sensor_arm_rgb':
                cam_prim = pr
                cam_path = str(pr.GetPath())
                break
    if cam_prim and cam_prim.IsActive():
        nodes.extend([
            ('ArmRp', 'isaacsim.core.nodes.IsaacCreateRenderProduct'),
            ('ArmRgb', 'isaacsim.ros2.bridge.ROS2CameraHelper'),
            ('ArmInfo', 'isaacsim.ros2.bridge.ROS2CameraInfoHelper'),
        ])
        conns.extend([
            ('Tick.outputs:tick', 'ArmRp.inputs:execIn'),
            ('ArmRp.outputs:execOut', 'ArmRgb.inputs:execIn'),
            ('ArmRp.outputs:execOut', 'ArmInfo.inputs:execIn'),
            ('ArmRp.outputs:renderProductPath', 'ArmRgb.inputs:renderProductPath'),
            ('ArmRp.outputs:renderProductPath', 'ArmInfo.inputs:renderProductPath'),
            ('Context.outputs:context', 'ArmRgb.inputs:context'),
            ('Context.outputs:context', 'ArmInfo.inputs:context'),
        ])
        vals.extend([
            ('ArmRp.inputs:cameraPrim', [Sdf.Path(cam_path)]),
            ('ArmRp.inputs:width', 640),
            ('ArmRp.inputs:height', 360),
            ('ArmRgb.inputs:type', 'rgb'),
            ('ArmRgb.inputs:topicName', '/arducam/image_rect'),
            ('ArmRgb.inputs:frameId', 'arducam_frame'),
            ('ArmRgb.inputs:frameSkipCount', 1),
            ('ArmInfo.inputs:topicName', '/arducam/camera_info'),
            ('ArmInfo.inputs:frameId', 'arducam_frame'),
            ('ArmInfo.inputs:frameSkipCount', 1),
        ])

    og.Controller.edit(
        {'graph_path': GRAPH, 'evaluator_name': 'execution'},
        {
            keys.CREATE_NODES: nodes,
            keys.CONNECT: conns,
            keys.SET_VALUES: vals,
        },
    )


# 로봇 발끝 높이에 맞춘 지면과 상자(205x205x70)·데모 퍽 2종 배치
def setup_stage():
    ext_mgr = omni.kit.app.get_app().get_extension_manager()
    ext_mgr.set_extension_enabled_immediate('isaacsim.ros2.bridge', True)
    omni.timeline.get_timeline_interface().stop()
    ctx = omni.usd.get_context()
    stage = ctx.get_stage()
    if not stage or not stage.GetPrimAtPath(ROBOT + '/base'):
        ctx.open_stage(USD_PATH)
        stage = ctx.get_stage()

    ps_prim = stage.GetPrimAtPath('/physicsScene')
    if not ps_prim:
        scene = UsdPhysics.Scene.Define(stage, '/physicsScene')
        scene.CreateGravityMagnitudeAttr(9.81)
        ps_prim = scene.GetPrim()
    ps_api = PhysxSchema.PhysxSceneAPI.Apply(ps_prim)
    ps_api.CreateEnableGPUDynamicsAttr(True)
    ps_api.CreateBroadphaseTypeAttr('GPU')
    ps_api.CreateSolverTypeAttr('TGS')
    ps_api.CreateTimeStepsPerSecondAttr(120)

    if stage.GetPrimAtPath('/MissionEnv'):
        stage.RemovePrim('/MissionEnv')
    UsdGeom.Xform.Define(stage, '/MissionEnv')
    dome = UsdLux.DomeLight.Define(stage, '/MissionEnv/DomeLight')
    dome.CreateIntensityAttr(480.0)
    mat = make_grip_mat(stage, '/MissionEnv/GripMat')
    tune_robot_physics(stage, mat)

    # 팔 ArduCam 외 미사용 카메라·거리 센서를 비활성화해 GUI RTX 프레임레이트 확보
    robot_prim = stage.GetPrimAtPath(ROBOT)
    if robot_prim:
        for pr in Usd.PrimRange(robot_prim):
            tname = pr.GetTypeName()
            if tname == 'Camera' and pr.GetName() != 'Sensor_arm_rgb':
                pr.SetActive(False)
            elif tname in ('Lidar', 'IsaacRtxLidarSensorAPI') or (
                'Sensor' in pr.GetName() and pr.GetName() != 'Sensor_arm_rgb'
                and tname not in ('Xform', 'Mesh', '')
            ):
                pr.SetActive(False)

    base_prim = stage.GetPrimAtPath(ROBOT + '/base')
    base_tf = UsdGeom.XformCache().GetLocalToWorldTransform(base_prim)
    bx, by, bz = base_tf.ExtractTranslation()
    gz = bz + GROUND_OFF_Z
    top_z = gz + 0.070

    add_static_box(
        stage, '/MissionEnv/Ground', (bx, by, gz - 0.010),
        (2.0, 2.0, 0.020), (0.25, 0.25, 0.27), mat,
    )
    add_static_box(
        stage, '/MissionEnv/WorkBox', (bx + BOX_X, by + BOX_Y, gz + 0.035),
        (0.205, 0.205, 0.070), (0.55, 0.42, 0.26), mat,
    )
    add_puck(
        stage, '/MissionEnv/RoundPuck', True,
        (bx + 0.265, by - 0.045, top_z + 0.0302), (0.0, 0.0, 0.0),
        (0.85, 0.20, 0.20), mat,
    )
    add_puck(
        stage, '/MissionEnv/SquareLyingPuck', False,
        (bx + 0.300, by + 0.040, top_z + 0.0152),
        (0.0, 90.0, math.degrees(0.25)),
        (0.20, 0.75, 0.30), mat,
    )

    build_ros2_graph(stage)
    print('Isaac Sim 미션 스테이지 준비 완료: Play 클릭 후 hardware:=isaac 실행')


setup_stage()
