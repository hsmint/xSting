# 작업·보관 관리 구현 위치
P1에서 task_manager·storage_manager·arm_executor를 함께 실행합니다.
RunTask → Nav2 → GetTarget → MoveArm 순서를 비동기로 연결합니다.
슬롯은 EMPTY/RESERVED/OCCUPIED/UNKNOWN 상태와 버전·예약 ID로 보호합니다.
현재는 실행 노드가 구현되지 않았습니다. 담당과 수락 조건은 docs/TODO_RL.md 및 docs/TEST_PLAN.md 참조.
