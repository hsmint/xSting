"""backend:=ros 실제 로봇(또는 시뮬레이터) 동작: 로봇 팀의 액션·서비스를 불러 행동 하나를 조립한다 (아직 구현 전)."""
import time

from ..core.contract import COMMANDS, Canceled


class RosBackend:
    def __init__(self, node, arena):
        self.node = node
        self.arena = arena
        # 아직 없는 행동(_ros_GOTO 등)은 NOT_IMPLEMENTED로 답한다
        self.handlers = {
            name: getattr(self, f"_ros_{name}", self._not_implemented)
            for name in COMMANDS}

    def stop(self):
        pass

    def _not_implemented(self, args, goal_handle):
        return {"success": False, "error": "NOT_IMPLEMENTED"}

    # ---------- 도우미 ----------
    def _wait_future(self, future, timeout_s, goal_handle=None):
        """콜백 안이라 spin_until_future_complete 대신 다른 스레드가 채우길 기다린다."""
        start = time.monotonic()
        while not future.done():
            if goal_handle is not None and goal_handle.is_cancel_requested:
                raise Canceled()
            if time.monotonic() - start > timeout_s:
                return False
            time.sleep(0.02)
        return True

    # ---------- 아직 구현 전 ----------
    # 행동마다 로봇 팀 액션·서비스를 부르고 결과를 contract.py 형식으로 바꾼다.
    # 새 액션 클라이언트는 callback_group=node.callback_group으로 만든다 (교착 방지).
    # TODO: _ros_GOTO  강화학습 주행에 장소 ID(또는 places.<t>.target)를 넘긴다.
    #       접근 자리는 로봇이 정한다. 결과에 robot.here, robot.pose(TF map → base_link),
    #       HOME이면 출발 구역 안 판정(in_zone)과 정지 시간(stopped_s)
    # TODO: _ros_READ_MISSION_INPUT  D435 영상 → Code128 + 신호 점멸 감지
    #       (못 찾으면 한 바퀴 돌며 찾기, rules.signal_errors에 걸리면 다음 주기
    #        다시 읽기, 처음 포함 3번 실패면 PERCEPTION_FAILED. contract.py 참고)
    # TODO: _ros_LOAD  AprilTag(퍽 id)·색·모양 판정 → 팔로 집어 적재함 빈 slot에
    # TODO: _ros_UNLOAD  items 순서대로 하차 박스 태그 기준 cell 위치에 놓고 2초 안정 확인
    # TODO: _ros_VERIFY_LOCATION  9칸 새 관측 → items·extra·cells
