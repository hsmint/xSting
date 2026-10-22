# 설정 소유권
현재 contracts.yaml은 개발 중 합의할 테스트 계약입니다. ROS 노드 파라미터 파일이 아니며 아직 노드에 자동 적용되지 않습니다.
운용 노드 구현 시 실제 파라미터 선언·검증 코드와 연결하고 YAML→노드 일치 테스트를 추가하세요.

- vision 담당: vision.yaml, recording.yaml의 영상 항목
- hardware 담당: robot.yaml, nav2.yaml, controllers.yaml, URDF·센서 TF
- RL 담당: tasks.yaml, arm.yaml, model.yaml, slots.yaml

장치 이름·실측 관절 제한·Nav2 플러그인·카메라 드라이버가 확정되기 전 임의 설정으로 채우지 않습니다.
Q2는 팔 단발 입력/출력의 reliable·volatile·depth 1, Q5는 주기 검출의 best_effort·volatile·depth 1입니다.
