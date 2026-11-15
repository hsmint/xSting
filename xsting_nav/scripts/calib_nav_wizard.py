#!/usr/bin/env python3
"""대화형 XSting 실기체 주행 보정 마법사 (xsting_nav calib_node 연동)."""

from __future__ import annotations

import argparse
from pathlib import Path
import subprocess
import sys

JOINT_LABELS = [
    "0: leg1_yaw  (ID 1,  앞-좌 Yaw  | +: 앞쪽 +X)",
    "1: leg1_hip  (ID 2,  앞-좌 Hip  | +: 허벅지 위로)",
    "2: leg1_knee (ID 3,  앞-좌 Knee | +: 발끝 위로)",
    "3: leg2_yaw  (ID 4,  중-좌 Yaw  | +: 앞쪽 +X)",
    "4: leg2_hip  (ID 5,  중-좌 Hip  | +: 허벅지 위로)",
    "5: leg2_knee (ID 6,  중-좌 Knee | +: 발끝 위로)",
    "6: leg3_yaw  (ID 7,  뒤-좌 Yaw  | +: 앞쪽 +X)",
    "7: leg3_hip  (ID 8,  뒤-좌 Hip  | +: 허벅지 위로)",
    "8: leg3_knee (ID 9,  뒤-좌 Knee | +: 발끝 위로)",
    "9: leg4_yaw  (ID 10, 앞-우 Yaw  | +: 앞쪽 +X)",
    "10: leg4_hip (ID 11, 앞-우 Hip  | +: 허벅지 위로)",
    "11: leg4_knee(ID 12, 앞-우 Knee | +: 발끝 위로)",
    "12: leg5_yaw (ID 13, 중-우 Yaw  | +: 앞쪽 +X)",
    "13: leg5_hip (ID 14, 중-우 Hip  | +: 허벅지 위로)",
    "14: leg5_knee(ID 15, 중-우 Knee | +: 발끝 위로)",
    "15: leg6_yaw (ID 16, 뒤-우 Yaw  | +: 앞쪽 +X)",
    "16: leg6_hip (ID 17, 뒤-우 Hip  | +: 허벅지 위로)",
    "17: leg6_knee(ID 18, 뒤-우 Knee | +: 발끝 위로)",
]


def default_calib_path() -> Path:
    src_cand = Path("/home/pa03/Documents/FINAL/mk2_ws/src/xsting_nav/config/calibration.yaml")
    if src_cand.exists():
        return src_cand
    return Path(__file__).resolve().parents[1] / "config" / "calibration.yaml"


def run_calib_cmd(calib_file: Path, extra_params: list[str]) -> int:
    cmd = [
        "ros2", "run", "xsting_nav", "calib_node",
        "--ros-args",
        "-p", f"calib_file:={calib_file}",
    ]
    for p in extra_params:
        cmd.extend(["-p", p])
    print(f"\n[실행] {' '.join(cmd)}")
    return subprocess.call(cmd)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--calib-file",
        type=Path,
        default=default_calib_path(),
        help="저장 및 갱신할 calibration.yaml 경로",
    )
    args = parser.parse_args()
    calib_file = args.calib_file.resolve()

    while True:
        print("\n" + "=" * 72)
        print("  XSting 실물 주행 보정 마법사 (xsting_nav calib_node)")
        print(f"  대상 보정 파일: {calib_file}")
        print("=" * 72)
        print("  [1] 전체 센서/관절 실시간 상태 점검 (status)")
        print("  [2] 1단계: IMU 수평 정렬 & 자이로 바이어스 보정 (imu)")
        print("  [3] 2단계-A: 18축 다리 영점(Zero/Stand) 오프셋 일괄 보정 (joints_zero)")
        print("  [4] 2단계-B: 개별 모터 조립 방향(+/-) 6° 미세 구동 검증 & 반전 (joints_nudge)")
        print("  [5] 2단계-C: 표준 서기 자세(STAND_POSE) 기립 검증 (stand_test)")
        print("  [6] 3단계-A: cuVSLAM 아레나 시작점(HOME: 0.734, 0.950, 180°) 정렬 (odom_origin)")
        print("  [7] 3단계-B: 카펫 보행 슬립 배율(vx/vy/wz scale) 보정 (gait_scale)")
        print("  [8] 4단계: nvBlox + LiDAR 4x4 ESDF 전방 벽 거리 검증/오프셋 보정 (esdf_check)")
        print("  [q] 종료")
        choice = input("\n메뉴 선택 > ").strip().lower()

        if choice == "q":
            print("보정 마법사를 종료합니다.")
            break
        if choice == "1":
            run_calib_cmd(calib_file, ["mode:=status", "duration_s:=2.0"])
        elif choice == "2":
            yaw = input("IMU 보드 장착 Yaw 각도 (기본 0.0, 예: 0 / 90 / -90 / 180) > ").strip() or "0.0"
            input("로봇을 평평한 바닥에 정지시킨 뒤 Enter를 누르세요 (5초 측정)...")
            run_calib_cmd(calib_file, ["mode:=imu", "duration_s:=5.0", f"board_yaw_deg:={yaw}"])
        elif choice == "3":
            ref = input("기준 자세 선택 [zero: 수평 영점 / stand: 표준 서기 지그] (기본 zero) > ").strip() or "zero"
            input(f"로봇 다리를 '{ref}' 기준 자세에 맞춘 뒤 Enter를 누르세요 (2.5초 평균 측정)...")
            run_calib_cmd(calib_file, ["mode:=joints_zero", f"ref_pose:={ref}", "duration_s:=2.5"])
        elif choice == "4":
            print("\n--- 관절 번호 목록 ---")
            for line in JOINT_LABELS:
                print("  " + line)
            jidx = input("검증할 관절 인덱스 (0~17, 기본 0) > ").strip() or "0"
            input("로봇 다리가 공중에 떠 있거나 안전한 상태인지 확인 후 Enter (+6° 미세 왕복)...")
            rc = run_calib_cmd(calib_file, ["mode:=joints_nudge", f"test_joint_index:={jidx}", "nudge_deg:=6.0"])
            if rc == 0:
                flip = input("방금 모터가 반대 방향으로 움직였나요? 부호(leg_sign)를 반전할까요? [y/N] > ").strip().lower()
                if flip == "y":
                    run_calib_cmd(calib_file, ["mode:=flip_joint_sign", f"test_joint_index:={jidx}"])
        elif choice == "5":
            hold = input("기립 후 유지 시간(초, 기본 5.0) > ").strip() or "5.0"
            input("주변 안전 확보 후 Enter를 누르면 2.5초에 걸쳐 부드럽게 서기 자세로 이동합니다...")
            run_calib_cmd(calib_file, ["mode:=stand_test", f"duration_s:={hold}"])
        elif choice == "6":
            tx = input("아레나 HOME X 좌표 (m, 기본 0.734) > ").strip() or "0.734"
            ty = input("아레나 HOME Y 좌표 (m, 기본 0.950) > ").strip() or "0.950"
            tyaw = input("아레나 HOME Yaw 각도 (deg, 기본 180.0) > ").strip() or "180.0"
            input("로봇을 아레나 HOME 출발 구역에 놓은 뒤 Enter를 누르세요...")
            run_calib_cmd(
                calib_file,
                [
                    "mode:=odom_origin",
                    "duration_s:=2.0",
                    f"target_arena_x:={tx}",
                    f"target_arena_y:={ty}",
                    f"target_arena_yaw_deg:={tyaw}",
                ],
            )
        elif choice == "7":
            axis = input("보정할 보행 축 선택 [vx / vy / wz] (기본 vx) > ").strip() or "vx"
            default_spd = "0.16" if axis == "vx" else ("0.10" if axis == "vy" else "0.50")
            spd = input(f"시험 속도 (기본 {default_spd}) > ").strip() or default_spd
            dur = input("보행 시간(초, 기본 3.0) > ").strip() or "3.0"
            meas = input("줄자 실측 거리/각도 사용 시 입력 (cuVSLAM 자동 측정 사용 시 Enter) > ").strip() or "0.0"
            input("전방/측면 1m 공간을 확보한 뒤 Enter를 누르세요...")
            run_calib_cmd(
                calib_file,
                [
                    "mode:=gait_scale",
                    f"gait_axis:={axis}",
                    f"gait_test_speed:={spd}",
                    f"duration_s:={dur}",
                    f"measured_delta:={meas}",
                ],
            )
        elif choice == "8":
            wall = input("전방 평면 벽까지 실제 거리(m, 단순 조회만 할 경우 0 입력) > ").strip() or "0.0"
            run_calib_cmd(calib_file, ["mode:=esdf_check", "duration_s:=2.0", f"known_front_wall_m:={wall}"])


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n중단되었습니다.")
        sys.exit(0)
