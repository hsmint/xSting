#!/usr/bin/env python3
import math
from pathlib import Path
import sys
import threading
import yaml

from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import Parameter as ParamMsg
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.srv import SetParameters
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from vision_msgs.msg import Detection3DArray
from xsting_interfaces.action import PickPlace

BOX_H = 0.070
GRASP_DZ = 0.053
CMD_RIGHT_ONLY = 5
CMD_VISION_PROBE = 6
ARM_JOINTS = (
    'arm_yaw',
    'arm_shoulder',
    'arm_elbow',
    'arm_wrist',
    'arm_roll',
    'finger_l',
)
SLOTS = (
    (0.0609, 0.0767, 0.163722, 2.12151),
    (0.0231, 0.0300, 0.162722, 2.80193),
    (0.0231, -0.0300, 0.162722, -2.80193),
    (0.0609, -0.0767, 0.163722, -2.12151),
    (-0.0112, 0.0771, 0.172722, 2.56746),
    (-0.0314, 0.0270, 0.172722, 2.95027),
    (-0.0314, -0.0270, 0.172722, -2.95027),
    (-0.0112, -0.0771, 0.172722, -2.56746),
    (-0.0660, 0.0788, 0.184722, 2.71635),
    (-0.0811, 0.0270, 0.184722, 2.99977),
    (-0.0811, -0.0270, 0.184722, -2.99977),
    (-0.0660, -0.0788, 0.184722, -2.71635),
)


# 워크스페이스 src 원본과 install 공유 경로의 pick_place.yaml 목록 탐색
def find_yaml_paths():
    share = Path(get_package_share_directory('xsting_pick_place'))
    paths = [share / 'config' / 'pick_place.yaml']
    for parent in Path(__file__).resolve().parents:
        found = list(
            (parent / 'src').glob('**/xsting_pick_place/config/pick_place.yaml')
        )
        if found:
            paths.insert(0, found[0])
            break
    return paths


# 팔 뻗음 반경(arm_yaw 기준)에 따른 처짐 보정 높이 계산
def calc_droop(cfg, x, y):
    reach = math.hypot(x - 0.108, y)
    return cfg['droop_k'] * max(0.0, reach - cfg['droop_r0'])


# 실기 픽앤플레이스 현장 보정 및 YAML 영구 저장 노드
class CalibNode(Node):

    def __init__(self):
        super().__init__('pick_place_calib')

        # 설정 파일 로드 및 내부 상태 초기화
        self._paths = find_yaml_paths()
        self._lock = threading.Lock()
        self._joints = {}
        self._det_msg = None
        self._cur_xyz = [0.285, 0.0, 0.160]
        self._cur_yaw = 0.0
        self._jog = [0.0, 0.0, 0.0]
        self._occ_slots = []
        self._cfg = {}
        self.load_cfg()

        # 관절·비전 구독 및 액션·파라미터 클라이언트 생성
        self.create_subscription(
            JointState, '/joint_states', self.on_joints, qos_profile_sensor_data
        )
        self.create_subscription(
            Detection3DArray, '/detections_3d', self.on_det, 10
        )
        self._pick_cli = ActionClient(self, PickPlace, '/pick_place')
        self._grip_cli = ActionClient(
            self,
            FollowJointTrajectory,
            '/gripper_trajectory_controller/follow_joint_trajectory',
        )
        self._param_cli = self.create_client(
            SetParameters, '/pick_place/set_parameters'
        )

    # YAML에서 보정 파라미터 읽기
    def load_cfg(self):
        raw = yaml.safe_load(self._paths[0].read_text(encoding='utf-8')) or {}
        p = raw.get('/**', {}).get('ros__parameters', {})
        self._cfg = {
            'pipeline': str(p.get('pipeline', 'isaac_ros_cumotion')),
            'vel_scale': float(p.get('vel_scale', 0.70)),
            'acc_scale': float(p.get('acc_scale', 0.60)),
            'ground_z': float(p.get('ground_z', -0.0213)),
            'box_x': float(p.get('box_x', 0.285)),
            'box_y': float(p.get('box_y', 0.0)),
            'auto_box_det': bool(p.get('auto_box_det', True)),
            'use_arducam': bool(p.get('use_arducam', True)),
            'arducam_look_dz': float(p.get('arducam_look_dz', 0.235)),
            'grip_open_box': float(p.get('grip_open_box', 0.0165)),
            'grip_open_slot': float(p.get('grip_open_slot', 0.0078)),
            'grip_loosen': float(p.get('grip_loosen', 0.00474)),
            'grip_close': float(p.get('grip_close', 0.0008)),
            'check_grasp': bool(p.get('check_grasp', True)),
            'grip_empty_th': float(p.get('grip_empty_th', 0.0025)),
            'box_hover_z': float(p.get('box_hover_z', 0.215)),
            'slot_hover_z': float(p.get('slot_hover_z', 0.318)),
            'pick_off_x': float(p.get('pick_off_x', 0.0)),
            'pick_off_y': float(p.get('pick_off_y', 0.0)),
            'pick_off_z': float(p.get('pick_off_z', 0.0)),
            'place_off_x': float(p.get('place_off_x', 0.0)),
            'place_off_y': float(p.get('place_off_y', 0.0)),
            'place_off_z': float(p.get('place_off_z', 0.0)),
            'slot_off_x': float(p.get('slot_off_x', 0.0)),
            'slot_off_y': float(p.get('slot_off_y', 0.0)),
            'slot_off_z': float(p.get('slot_off_z', 0.0)),
            'right_tip_r': float(p.get('right_tip_r', 0.0190)),
            'right_grasp_dz': float(p.get('right_grasp_dz', 0.0210)),
            'right_lift_dz': float(p.get('right_lift_dz', 0.0880)),
            'right_off_z': float(p.get('right_off_z', 0.0)),
            'droop_r0': float(p.get('droop_r0', 0.200)),
            'droop_k': float(p.get('droop_k', 0.0)),
        }

    # 최신 관절 각도 저장
    def on_joints(self, msg):
        with self._lock:
            for i, name in enumerate(msg.name):
                if i < len(msg.position):
                    self._joints[name] = float(msg.position[i])

    # 최신 3D 비전 검출 메시지 저장
    def on_det(self, msg):
        with self._lock:
            self._det_msg = msg

    # 실행 중인 /pick_place 노드에 보정 파라미터 즉시 반영
    def sync_params(self, sync_slots=False):
        if not self._param_cli.wait_for_service(timeout_sec=1.5):
            print('[경고] /pick_place 파라미터 서비스를 찾을 수 없습니다.')
            return False

        keys = (
            'vel_scale',
            'acc_scale',
            'ground_z',
            'box_x',
            'box_y',
            'arducam_look_dz',
            'grip_open_box',
            'grip_open_slot',
            'grip_loosen',
            'grip_close',
            'grip_empty_th',
            'box_hover_z',
            'slot_hover_z',
            'pick_off_x',
            'pick_off_y',
            'pick_off_z',
            'place_off_x',
            'place_off_y',
            'place_off_z',
            'slot_off_x',
            'slot_off_y',
            'slot_off_z',
            'right_tip_r',
            'right_grasp_dz',
            'right_lift_dz',
            'right_off_z',
            'droop_r0',
            'droop_k',
        )
        req = SetParameters.Request()
        for key in keys:
            msg = ParamMsg()
            msg.name = key
            msg.value = ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE,
                double_value=float(self._cfg[key]),
            )
            req.parameters.append(msg)
        for bkey in ('auto_box_det', 'use_arducam', 'check_grasp'):
            msg = ParamMsg()
            msg.name = bkey
            msg.value = ParameterValue(
                type=ParameterType.PARAMETER_BOOL,
                bool_value=bool(self._cfg[bkey]),
            )
            req.parameters.append(msg)

        if sync_slots:
            occ_msg = ParamMsg()
            occ_msg.name = 'occupied_slots'
            occ_msg.value = ParameterValue(
                type=ParameterType.PARAMETER_INTEGER_ARRAY,
                integer_array_value=[int(s) for s in self._occ_slots],
            )
            req.parameters.append(occ_msg)

        done = threading.Event()
        fut = self._param_cli.call_async(req)
        fut.add_done_callback(lambda _: done.set())
        done.wait(timeout=3.0)
        print('[적용] /pick_place 실시간 파라미터 동기화 완료')
        return True

    # /pick_place 액션 목표 전송 및 완료 대기
    def send_pick(self, goal):
        if not self._pick_cli.wait_for_server(timeout_sec=2.0):
            print('[오류] /pick_place 액션 서버가 실행 중이지 않습니다.')
            return None

        ev = threading.Event()
        fut = self._pick_cli.send_goal_async(goal)
        fut.add_done_callback(lambda _: ev.set())
        if not ev.wait(timeout=5.0) or not fut.result() or not fut.result().accepted:
            print('[오류] 액션 목표가 거절되었습니다.')
            return None

        res_ev = threading.Event()
        res_fut = fut.result().get_result_async()
        res_fut.add_done_callback(lambda _: res_ev.set())
        if not res_ev.wait(timeout=60.0):
            print('[오류] 동작 완료 대기 시간 초과')
            return None
        return res_fut.result().result

    # 홈 자세 복귀
    def move_home(self):
        goal = PickPlace.Goal()
        goal.command = PickPlace.Goal.CMD_HOME
        res = self.send_pick(goal)
        if res:
            print(f'[홈] 완료={res.success} 메시지="{res.message}"')

    # 하드웨어 접힌 홈(kArmHome) 및 직립 영점(0 rad) 대비 관절 각도 확인
    def check_zero(self):
        for _ in range(20):
            with self._lock:
                meas = dict(self._joints)
            if all(j in meas for j in ARM_JOINTS):
                break
            threading.Event().wait(0.05)
        missing = [j for j in ARM_JOINTS if j not in meas]
        if missing:
            print(f'[오류] /joint_states 미수신 관절: {missing}')
            return

        hw_home = {
            'arm_yaw': 0.0,
            'arm_shoulder': -1.0508,
            'arm_elbow': 1.0707,
            'arm_wrist': 0.0,
            'arm_roll': 0.0,
            'finger_l': 0.0,
        }
        print('[관절 확인] 현재 측정값 및 하드웨어 접힌 홈(kArmHome) 대비 오차:')
        for name in ARM_JOINTS:
            val = meas[name]
            ref = hw_home[name]
            diff = val - ref
            if name == 'finger_l':
                print(f'  {name:14s}: {val:+.5f} m (홈 대비 {diff*1000:+.2f} mm)')
            else:
                print(
                    f'  {name:14s}: {val:+.5f} rad ({math.degrees(val):+6.2f} deg) '
                    f'[홈 오차 {math.degrees(diff):+5.2f} deg]'
                )

    # 지정된 base 기준 패드 좌표로 보정 프로브 이동
    def probe_xyz(self, x, y, z, yaw=0.0):
        self._cur_xyz = [float(x), float(y), float(z)]
        self._cur_yaw = float(yaw)
        goal = PickPlace.Goal()
        goal.command = PickPlace.Goal.CMD_PROBE
        goal.probe_xyz = [float(x), float(y), float(z)]
        goal.probe_yaw = float(yaw)
        res = self.send_pick(goal)
        if res and len(res.final_xyz) >= 3:
            fx, fy, fz = res.final_xyz[:3]
            print(
                f'[프로브] 완료={res.success} '
                f'명령=({x:.4f}, {y:.4f}, {z:.4f}) '
                f'도달=({fx:.4f}, {fy:.4f}, {fz:.4f})'
            )

    # 205x205x70 상자 중심 파지점 상공으로 프로브 이동
    def probe_box(self, hover=0.025):
        self._jog = [0.0, 0.0, 0.0]
        box_top = self._cfg['ground_z'] + BOX_H
        x = self._cfg['box_x'] + self._cfg['pick_off_x']
        y = self._cfg['box_y'] + self._cfg['pick_off_y']
        z = (
            box_top
            + GRASP_DZ
            + self._cfg['pick_off_z']
            + calc_droop(self._cfg, x, y)
            + hover
        )
        self.probe_xyz(x, y, z, 0.0)

    # 상자 중앙에 누운 60mm 사각 퍽의 먼 끝 편심 파지점으로 프로브 이동
    def probe_lying(self, hover=0.020, yaw_deg=0.0):
        self.sync_params()
        self._jog = [0.0, 0.0, 0.0]
        lyaw = math.radians(float(yaw_deg))
        box_top = self._cfg['ground_z'] + BOX_H
        x = (
            self._cfg['box_x']
            + self._cfg['right_tip_r'] * math.cos(lyaw)
            + self._cfg['pick_off_x']
        )
        y = (
            self._cfg['box_y']
            + self._cfg['right_tip_r'] * math.sin(lyaw)
            + self._cfg['pick_off_y']
        )
        z = (
            box_top
            + self._cfg['right_grasp_dz']
            + calc_droop(self._cfg, x, y)
            + hover
        )
        print(
            f'[누운 퍽 프로브] yaw={yaw_deg:+.1f}deg '
            f'tip_r={self._cfg["right_tip_r"]*1000:.1f}mm '
            f'grasp_dz={self._cfg["right_grasp_dz"]*1000:.1f}mm '
            f'hover={hover*1000:+.1f}mm'
        )
        self.probe_xyz(x, y, z, lyaw)

    # ArduCam YOLOv8(2-Pass 정렬) 또는 /detections_3d로 직립·누운 퍽 파지점 프로브
    def probe_vision(self, hover=0.025, only_lying=False):
        self.sync_params()
        self._jog = [0.0, 0.0, 0.0]
        goal = PickPlace.Goal()
        goal.command = CMD_VISION_PROBE
        goal.probe_xyz = [float(hover)]
        goal.target_class = 'lying' if only_lying else ''
        res = self.send_pick(goal)
        if not res:
            return
        if not res.success or len(res.final_xyz) < 10:
            print(f'[비전 프로브 실패] {res.message}')
            return

        mx, my, mz, cx, cy, cz, pyaw, rx, ry, rz = res.final_xyz[:10]
        self._cur_xyz = [float(cx), float(cy), float(cz)]
        self._cur_yaw = float(pyaw)
        cls_name = res.detected_classes[0] if res.detected_classes else 'unknown'
        state_str = '누움(lying)' if res.righted_count > 0 else '직립(standing)'
        print(
            f'[비전 프로브] 상태={state_str} cls={cls_name} '
            f'yaw={math.degrees(pyaw):+.1f}deg\n'
            f'  원검출=({rx:.4f}, {ry:.4f}, {rz:.4f}) '
            f'명령=({cx:.4f}, {cy:.4f}, {cz:.4f}) '
            f'도달=({mx:.4f}, {my:.4f}, {mz:.4f})'
        )

    # 바스켓 슬롯(1~12) 상공 또는 내부로 프로브 이동
    def probe_slot(self, slot, hover=0.030):
        idx = max(1, min(12, int(slot))) - 1
        self._jog = [0.0, 0.0, 0.0]
        sx, sy, sz, syaw = SLOTS[idx]
        x = sx + self._cfg['slot_off_x']
        y = sy + self._cfg['slot_off_y']
        z = sz + GRASP_DZ + self._cfg['slot_off_z'] + hover
        self.probe_xyz(x, y, z, syaw)

    # 현재 프로브 위치에서 (dx, dy, dz) 미터만큼 미세 조그 이동
    def jog_pad(self, dx, dy, dz):
        self._jog[0] += dx
        self._jog[1] += dy
        self._jog[2] += dz
        print(
            f'[조그 누적] dx={self._jog[0]:+.4f} '
            f'dy={self._jog[1]:+.4f} dz={self._jog[2]:+.4f}'
        )
        self.probe_xyz(
            self._cur_xyz[0] + dx,
            self._cur_xyz[1] + dy,
            self._cur_xyz[2] + dz,
            self._cur_yaw,
        )

    # 누적 조그량을 pick·place·slot·lying·right·droop 파라미터에 반영
    def apply_jog(self, target):
        if target == 'pick':
            self._cfg['pick_off_x'] += self._jog[0]
            self._cfg['pick_off_y'] += self._jog[1]
            self._cfg['pick_off_z'] += self._jog[2]
        elif target == 'place':
            self._cfg['place_off_x'] += self._jog[0]
            self._cfg['place_off_y'] += self._jog[1]
            self._cfg['place_off_z'] += self._jog[2]
        elif target == 'slot':
            self._cfg['slot_off_x'] += self._jog[0]
            self._cfg['slot_off_y'] += self._jog[1]
            self._cfg['slot_off_z'] += self._jog[2]
        elif target == 'lying':
            d_tip = (
                self._jog[0] * math.cos(self._cur_yaw)
                + self._jog[1] * math.sin(self._cur_yaw)
            )
            self._cfg['right_tip_r'] = max(
                0.008, min(0.028, self._cfg['right_tip_r'] + d_tip)
            )
            self._cfg['right_grasp_dz'] = max(
                0.010, min(0.040, self._cfg['right_grasp_dz'] + self._jog[2])
            )
            print(
                f'[누운 퍽 보정] right_tip_r={self._cfg["right_tip_r"]:.5f}m '
                f'right_grasp_dz={self._cfg["right_grasp_dz"]:.5f}m'
            )
        elif target == 'right':
            self._cfg['right_off_z'] += self._jog[2]
        elif target == 'droop':
            reach = math.hypot(self._cur_xyz[0] - 0.108, self._cur_xyz[1])
            span = reach - self._cfg['droop_r0']
            if span <= 0.020:
                print('[오류] droop 보정은 전방 상자 프로브 상태에서 수행하세요.')
                return
            self._cfg['droop_k'] += self._jog[2] / span
        else:
            print(
                '[오류] apply 대상은 pick, place, slot, lying, right, droop 중 하나입니다.'
            )
            return

        self._jog = [0.0, 0.0, 0.0]
        self.sync_params()

    # 그리퍼 프리셋(box, slot, loosen, close) 또는 직접 수치(m) 구동 테스트
    def test_grip(self, mode):
        presets = {
            'box': self._cfg['grip_open_box'],
            'slot': self._cfg['grip_open_slot'],
            'loosen': self._cfg['grip_loosen'],
            'close': self._cfg['grip_close'],
        }
        width = presets[mode] if mode in presets else float(mode)
        if not self._grip_cli.wait_for_server(timeout_sec=2.0):
            print('[오류] 그리퍼 컨트롤러 액션 서버를 찾을 수 없습니다.')
            return

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['finger_l']
        pt = JointTrajectoryPoint()
        pt.positions = [float(width)]
        pt.time_from_start = Duration(seconds=0.5).to_msg()
        goal.trajectory.points.append(pt)

        ev = threading.Event()
        fut = self._grip_cli.send_goal_async(goal)
        fut.add_done_callback(lambda _: ev.set())
        if ev.wait(timeout=3.0) and fut.result() and fut.result().accepted:
            res_ev = threading.Event()
            res_fut = fut.result().get_result_async()
            res_fut.add_done_callback(lambda _: res_ev.set())
            res_ev.wait(timeout=2.5)
        threading.Event().wait(0.15)
        with self._lock:
            meas = self._joints.get('finger_l', width)
        cmd_gap = (19.9 + 2.0 * width * 1000.0)
        meas_gap = (19.9 + 2.0 * meas * 1000.0)
        held = '파지됨' if meas >= self._cfg['grip_empty_th'] else '빈손'
        print(
            f'[그리퍼] 명령={width:.5f}m({cmd_gap:.1f}mm) -> '
            f'실측={meas:.5f}m({meas_gap:.1f}mm) [{held}, 임계={self._cfg["grip_empty_th"]:.4f}m]'
        )

    # 상자 위 누운 사각 퍽 단독 기립 시험(슬롯 적재 생략)
    def run_right_only(self):
        self.sync_params()
        goal = PickPlace.Goal()
        goal.command = CMD_RIGHT_ONLY
        goal.occupied_slots = [int(s) for s in self._occ_slots]
        res = self.send_pick(goal)
        if res:
            print(
                f'[기립 단독 시험] 완료={res.success} 기립={res.righted_count} '
                f'메시지="{res.message}"'
            )

    # 실기 적재 또는 하역 1회 동작 검증
    def run_trial(self, is_load, slot):
        self.sync_params()
        goal = PickPlace.Goal()
        goal.command = (
            PickPlace.Goal.CMD_LOAD_BOX
            if is_load
            else PickPlace.Goal.CMD_UNLOAD_BOX
        )
        goal.slots = [max(1, min(12, int(slot)))]
        goal.max_count = 1
        res = self.send_pick(goal)
        if res:
            if res.success:
                s_id = goal.slots[0]
                if is_load and s_id not in self._occ_slots:
                    self._occ_slots = sorted(self._occ_slots + [s_id])
                elif not is_load and s_id in self._occ_slots:
                    self._occ_slots.remove(s_id)
            print(
                f'[시험] 완료={res.success} 처리={res.handled_count} '
                f'기립={res.righted_count} 메시지="{res.message}"'
            )

    # 보정된 파라미터를 src 및 install 경로의 pick_place.yaml에 영구 저장
    def save_cfg(self):
        c = self._cfg
        auto_str = 'true' if c['auto_box_det'] else 'false'
        ardu_str = 'true' if c['use_arducam'] else 'false'
        chk_str = 'true' if c['check_grasp'] else 'false'
        text = (
            '/**:\n'
            '  ros__parameters:\n'
            '    # 계획 파이프라인(cumotion 우선 사용, 미설치 시 ompl 자동 전환)\n'
            f'    pipeline: "{c["pipeline"]}"\n'
            f'    vel_scale: {c["vel_scale"]:.2f}\n'
            f'    acc_scale: {c["acc_scale"]:.2f}\n\n'
            '    # 지면 높이 및 205x205x70 상자 기본 중심 좌표(base 프레임 기준, m)\n'
            f'    ground_z: {c["ground_z"]:.5f}\n'
            f'    box_x: {c["box_x"]:.5f}\n'
            f'    box_y: {c["box_y"]:.5f}\n'
            f'    auto_box_det: {auto_str}\n\n'
            '    # 팔 ArduCam Eye-in-Hand 수직 관측 정밀 보정 및 상판 대비 카메라 높이(m)\n'
            f'    use_arducam: {ardu_str}\n'
            f'    arducam_look_dz: {c["arducam_look_dz"]:.4f}\n\n'
            '    # 그리퍼 개폐 위치(finger_l, m: 간격=19.9mm+2*q) 및 빈손 판정 임계치\n'
            f'    grip_open_box: {c["grip_open_box"]:.5f}\n'
            f'    grip_open_slot: {c["grip_open_slot"]:.5f}\n'
            f'    grip_loosen: {c["grip_loosen"]:.5f}\n'
            f'    grip_close: {c["grip_close"]:.5f}\n'
            f'    check_grasp: {chk_str}\n'
            f'    grip_empty_th: {c["grip_empty_th"]:.5f}\n\n'
            '    # 안전 호버 높이(base 기준 패드 중심 Z, m)\n'
            f'    box_hover_z: {c["box_hover_z"]:.4f}\n'
            f'    slot_hover_z: {c["slot_hover_z"]:.4f}\n\n'
            '    # 실기 파지·배치·슬롯·기립 미세 오프셋(m) 및 팔 처짐 보정 계수\n'
            f'    pick_off_x: {c["pick_off_x"]:.5f}\n'
            f'    pick_off_y: {c["pick_off_y"]:.5f}\n'
            f'    pick_off_z: {c["pick_off_z"]:.5f}\n'
            f'    place_off_x: {c["place_off_x"]:.5f}\n'
            f'    place_off_y: {c["place_off_y"]:.5f}\n'
            f'    place_off_z: {c["place_off_z"]:.5f}\n'
            f'    slot_off_x: {c["slot_off_x"]:.5f}\n'
            f'    slot_off_y: {c["slot_off_y"]:.5f}\n'
            f'    slot_off_z: {c["slot_off_z"]:.5f}\n'
            f'    right_tip_r: {c["right_tip_r"]:.5f}\n'
            f'    right_grasp_dz: {c["right_grasp_dz"]:.5f}\n'
            f'    right_lift_dz: {c["right_lift_dz"]:.5f}\n'
            f'    right_off_z: {c["right_off_z"]:.5f}\n'
            f'    droop_r0: {c["droop_r0"]:.4f}\n'
            f'    droop_k: {c["droop_k"]:.5f}\n'
        )
        for path in self._paths:
            path.write_text(text, encoding='utf-8')
            print(f'[저장 완료] {path}')

    # 현재 보정 상태 요약 출력
    def show_state(self):
        c = self._cfg
        print(
            f'  speed    = vel:{c["vel_scale"]:.2f} acc:{c["acc_scale"]:.2f}  '
            f'occupied_slots = {self._occ_slots}\n'
            f'  ground_z = {c["ground_z"]:+.5f}  '
            f'box_xy = ({c["box_x"]:.4f}, {c["box_y"]:+.4f})  '
            f'arducam = {c["use_arducam"]} (dz={c["arducam_look_dz"]:.3f})\n'
            f'  pick_off = ({c["pick_off_x"]:+.4f}, {c["pick_off_y"]:+.4f}, {c["pick_off_z"]:+.4f})\n'
            f'  place_off= ({c["place_off_x"]:+.4f}, {c["place_off_y"]:+.4f}, {c["place_off_z"]:+.4f})\n'
            f'  slot_off = ({c["slot_off_x"]:+.4f}, {c["slot_off_y"]:+.4f}, {c["slot_off_z"]:+.4f})\n'
            f'  lying    = tip_r:{c["right_tip_r"]:.4f} grasp_dz:{c["right_grasp_dz"]:.4f} '
            f'lift_dz:{c["right_lift_dz"]:.4f} right_z:{c["right_off_z"]:+.4f} '
            f'droop_k:{c["droop_k"]:+.4f}\n'
            f'  grip     = box:{c["grip_open_box"]:.4f} slot:{c["grip_open_slot"]:.4f} '
            f'loosen:{c["grip_loosen"]:.4f} close:{c["grip_close"]:.4f} '
            f'empty_th:{c["grip_empty_th"]:.4f} (check={c["check_grasp"]})\n'
            f'  jog      = ({self._jog[0]:+.4f}, {self._jog[1]:+.4f}, {self._jog[2]:+.4f})'
        )


# 대화형 보정 메뉴 출력
def print_help():
    print(
        '\n=== xSting 실측 보정 CLI (calib_pick_place.py) ===\n'
        '  home                       : 하드웨어 접힌 홈 자세 복귀\n'
        '  zero                       : 현재 팔 6관절 및 접힌 홈 대비 오차 확인\n'
        '  box [hover_m]              : 상자 중심 직립 퍽 파지점 프로브 (기본 +0.025m)\n'
        '  lying [hover_m] [yaw_deg]  : 상자 중심 누운 퍽 먼 끝 파지점 프로브 (기본 +0.020m, 0deg)\n'
        '  vision [lying] [hover_m]   : ArduCam/3D비전 직립·누운 퍽 자동 검출 프로브\n'
        '  slot <1~12> [hover_m]      : 바스켓 슬롯 프로브 (기본 +0.030m)\n'
        '  slots [clear | 1 2 ...]    : 바스켓 기적재 퍽 충돌 슬롯 조회·초기화·지정\n'
        '  jog <dx> <dy> <dz>         : 패드 미세 이동(m, 예: jog 0 -0.002 -0.004)\n'
        '  apply <pick|place|slot|lying|right|droop> : 누적 조그를 보정치에 반영\n'
        '  grip <box|slot|loosen|close|값>     : 그리퍼 폭 및 파지 판정 즉시 시험\n'
        '  set <key> <val>            : 단일 파라미터 즉시 변경 (예: set grip_loosen 0.005)\n'
        '  right                      : 상자 위 누운 사각 퍽 단독 기립 시험(적재 생략)\n'
        '  load <1~12> / unload <1~12>: 실기 1개 적재(기립 포함) / 하역 시험\n'
        '  show / save / quit         : 현재 값 조회 / pick_place.yaml 저장 / 종료\n'
    )


# 한 줄 명령어를 파싱해 보정 동작 수행
def run_cmd(node, line):
    parts = line.strip().split()
    if not parts:
        return True
    cmd = parts[0].lower()

    if cmd in ('quit', 'exit', 'q'):
        return False
    if cmd == 'help':
        print_help()
    elif cmd == 'show':
        node.show_state()
    elif cmd == 'home':
        node.move_home()
    elif cmd == 'zero':
        node.check_zero()
    elif cmd == 'box':
        hover = float(parts[1]) if len(parts) > 1 else 0.025
        node.probe_box(hover)
    elif cmd == 'lying':
        hover = float(parts[1]) if len(parts) > 1 else 0.020
        yaw_deg = float(parts[2]) if len(parts) > 2 else 0.0
        node.probe_lying(hover, yaw_deg)
    elif cmd == 'vision':
        only_lying = len(parts) > 1 and parts[1].lower() == 'lying'
        h_idx = 2 if only_lying else 1
        hover = float(parts[h_idx]) if len(parts) > h_idx else 0.025
        node.probe_vision(hover, only_lying)
    elif cmd == 'right':
        node.run_right_only()
    elif cmd == 'slot' and len(parts) >= 2:
        hover = float(parts[2]) if len(parts) > 2 else 0.030
        node.probe_slot(int(parts[1]), hover)
    elif cmd == 'slots':
        if len(parts) == 1:
            print(f'[기적재 슬롯] {node._occ_slots}')
        elif parts[1].lower() == 'clear':
            node._occ_slots = []
            node.sync_params(sync_slots=True)
        else:
            node._occ_slots = sorted({
                int(x) for x in parts[1:] if 1 <= int(x) <= 12
            })
            node.sync_params(sync_slots=True)
    elif cmd == 'jog' and len(parts) == 4:
        node.jog_pad(float(parts[1]), float(parts[2]), float(parts[3]))
    elif cmd == 'apply' and len(parts) == 2:
        node.apply_jog(parts[1].lower())
    elif cmd == 'grip' and len(parts) == 2:
        node.test_grip(parts[1].lower())
    elif cmd == 'set' and len(parts) == 3:
        key = parts[1]
        if key in node._cfg and isinstance(node._cfg[key], bool):
            node._cfg[key] = parts[2].lower() in ('1', 'true', 'on', 'yes')
            node.sync_params()
        elif key in node._cfg and isinstance(node._cfg[key], float):
            node._cfg[key] = float(parts[2])
            node.sync_params()
        else:
            print(f'[오류] 변경 가능한 파라미터가 아닙니다: {key}')
    elif cmd == 'load' and len(parts) == 2:
        node.run_trial(True, int(parts[1]))
    elif cmd == 'unload' and len(parts) == 2:
        node.run_trial(False, int(parts[1]))
    elif cmd == 'save':
        node.save_cfg()
    else:
        print('[오류] 알 수 없는 명령입니다. help를 입력하세요.')
    return True


# 단발 인자 실행 또는 대화형 보정 루프 구동
def main():
    rclpy.init()
    node = CalibNode()
    spin_th = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_th.start()

    try:
        if len(sys.argv) > 1:
            run_cmd(node, ' '.join(sys.argv[1:]))
            return

        print_help()
        node.show_state()
        while rclpy.ok():
            try:
                line = input('calib> ')
            except EOFError:
                break
            if not run_cmd(node, line):
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
