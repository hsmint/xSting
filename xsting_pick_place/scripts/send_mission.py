#!/usr/bin/env python3
import argparse
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from xsting_interfaces.action import PickPlace


# 픽앤플레이스 액션 클라이언트를 통해 명령 전송 및 피드백 출력
class MissionCli(Node):

    def __init__(self):
        super().__init__('mission_cli')
        self._cli = ActionClient(self, PickPlace, '/pick_place')

    # 액션 목표를 전송하고 완료될 때까지 대기
    def send_goal(self, goal):
        if not self._cli.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('/pick_place 액션 서버를 찾을 수 없습니다')
            return 1

        fut = self._cli.send_goal_async(goal, feedback_callback=self.on_fb)
        rclpy.spin_until_future_complete(self, fut)
        gh = fut.result()
        if not gh or not gh.accepted:
            self.get_logger().error('액션 목표가 거절되었습니다')
            return 1

        res_fut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut)
        res = res_fut.result().result
        self.get_logger().info(
            f'완료={res.success} 처리={res.handled_count} '
            f'기립={res.righted_count} 슬롯={list(res.used_slots)} '
            f'메시지="{res.message}"'
        )
        return 0 if res.success else 1

    # 진행 단계 피드백 출력
    def on_fb(self, msg):
        fb = msg.feedback
        self.get_logger().info(
            f'[{fb.current_idx}/{fb.total_count}] 단계: {fb.phase}'
        )


# 명령줄 인자 파싱 및 모드별 액션 목표 구성
def main():
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest='cmd', required=True)

    sub.add_parser('home')

    p_right = sub.add_parser('right')
    p_right.add_argument('--demo', action='store_true')
    p_right.add_argument('--occupied', type=int, nargs='*', default=[])

    p_load = sub.add_parser('load')
    p_load.add_argument('--slots', type=int, nargs='+', default=[1, 2])
    p_load.add_argument('--occupied', type=int, nargs='*', default=[])
    p_load.add_argument('--count', type=int, default=2)
    p_load.add_argument('--cls', type=str, default='')
    p_load.add_argument('--demo', action='store_true')

    p_unload = sub.add_parser('unload')
    p_unload.add_argument('--slots', type=int, nargs='+', default=[1, 2])
    p_unload.add_argument('--occupied', type=int, nargs='*', default=[])
    p_unload.add_argument('--eye', type=int, default=0)

    args = parser.parse_args()
    goal = PickPlace.Goal()

    # 모드별 Goal 필드 채우기
    if args.cmd == 'home':
        goal.command = PickPlace.Goal.CMD_HOME
    elif args.cmd == 'right':
        goal.command = 5
        goal.occupied_slots = args.occupied
        if args.demo:
            goal.use_manual = True
            goal.puck_x = [0.295]
            goal.puck_y = [0.035]
            goal.puck_z = [0.0637]
            goal.puck_yaw = [0.25]
            goal.is_square = [True]
            goal.is_lying = [True]
            goal.puck_classes = ['green_square']
    elif args.cmd == 'load':
        goal.command = PickPlace.Goal.CMD_LOAD_BOX
        goal.slots = args.slots
        goal.occupied_slots = args.occupied
        goal.max_count = args.count
        goal.target_class = args.cls
        if args.demo:
            goal.use_manual = True
            goal.puck_x = [0.265, 0.300]
            goal.puck_y = [-0.045, 0.040]
            goal.puck_z = [0.0787, 0.0637]
            goal.puck_yaw = [0.0, 0.25]
            goal.is_square = [False, True]
            goal.is_lying = [False, True]
            goal.puck_classes = ['red_cylinder', 'green_square']
    elif args.cmd == 'unload':
        goal.command = PickPlace.Goal.CMD_UNLOAD_BOX
        goal.slots = args.slots
        goal.occupied_slots = args.occupied
        goal.dice_eye = args.eye

    rclpy.init()
    cli = MissionCli()
    code = cli.send_goal(goal)
    cli.destroy_node()
    rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
