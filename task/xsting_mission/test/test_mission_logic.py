"""주사위 기억 로봇 미션 규칙·할당·상태 가드·ArduCam 9칸 매칭 자동화 단위 테스트."""
import math
from types import SimpleNamespace

import pytest

from xsting_mission.core import contract, planner, rules, state
from xsting_mission.robot.mission_bridge import (
    MissionBridge,
    _base_to_map_xy,
    _cell_map_offset,
    _map_to_base_xy,
)


def _load_all_12_pucks(st, specs):
    """specs: 12개 (color, shape) 리스트를 S1~S6 순서대로 2개씩 LOAD 반영한다."""
    for box_idx, box_id in enumerate(rules.START_BOXES):
        state.apply_result(
            st,
            "GOTO",
            {"t": box_id},
            {"success": True, "robot": {"here": box_id, "pose": {"x": 1000, "y": 950, "yaw": 180}}},
        )
        p0 = 2 * box_idx
        p1 = p0 + 1
        state.apply_result(
            st,
            "LOAD",
            {},
            {
                "success": True,
                "box_remaining": 0,
                "pucks": [
                    {
                        "slot": p0,
                        "puck": {"id": p0, "color": specs[p0][0], "shape": specs[p0][1], "conf": 0.95},
                    },
                    {
                        "slot": p1,
                        "puck": {"id": p1, "color": specs[p1][0], "shape": specs[p1][1], "conf": 0.93},
                    },
                ],
            },
        )


def test_signal_validation_and_tie_breaking():
    seq_q1 = list("RGRRGRGGYGYG")
    assert rules.signal_errors(seq_q1, end_seen=True) == []
    assert rules.count_signal(seq_q1) == {"R": 4, "Y": 2, "G": 6}

    seq_tie_rg = list("RGRRGRRGYGYG")
    assert rules.signal_errors(seq_tie_rg, end_seen=True) == []
    assert rules.count_signal(seq_tie_rg) == {"R": 5, "Y": 2, "G": 5}

    # 끝 소등 미확인, 12회 미만, 1~6 범위 이탈 오류 검출
    assert "END_DARK_NOT_SEEN" in rules.signal_errors(seq_q1, end_seen=False)
    assert "SIGNAL_COUNT_10" in rules.signal_errors(seq_q1[:10], end_seen=True)
    assert "COUNT_OUT_OF_RANGE_R_7" in rules.signal_errors(
        list("RRRRRRRYYYYG"), end_seen=True
    )

    # P2-01(오름차순): Y(2) < R(5) == G(5) -> R→Y→G 동률 우선순위 유지: L1=Y, L2=R, L3=G
    st = state.new_state()
    state.apply_result(
        st,
        "READ_MISSION_INPUT",
        {},
        {"success": True, "barcode": "P2-01", "signal": seq_tie_rg, "end_seen": True},
    )
    assert planner.group_order(st) == {"L1": "Y", "L2": "R", "L3": "G"}

    # P2-02(내림차순) & R=4, Y=4, G=4 완전 동률 -> R→Y→G 그대로 L1=R, L2=Y, L3=G
    st_tie = state.new_state()
    state.apply_result(
        st_tie,
        "READ_MISSION_INPUT",
        {},
        {"success": True, "barcode": "P2-02", "signal": list("RYGRYGRYGRYG"), "end_seen": True},
    )
    assert planner.group_order(st_tie) == {"L1": "R", "L2": "Y", "L3": "G"}


def test_planner_assign_valid_and_impossible_cases():
    # Case 1 (Q1 기준): P2-01(asc), R=4, Y=2, G=6, 빨강 원기둥 3+사각 3, 초록 원기둥 3+사각 3
    st = state.new_state()
    state.apply_result(
        st,
        "READ_MISSION_INPUT",
        {},
        {"success": True, "barcode": "P2-01", "signal": list("RGRRGRGGYGYG"), "end_seen": True},
    )
    specs_q1 = [
        ("R", "cylinder"), ("G", "box"),
        ("R", "box"), ("G", "cylinder"),
        ("R", "box"), ("R", "cylinder"),
        ("G", "cylinder"), ("G", "box"),
        ("R", "cylinder"), ("G", "cylinder"),
        ("R", "box"), ("G", "box"),
    ]
    _load_all_12_pucks(st, specs_q1)
    plan = planner.assign(st)
    assert [item["cell"] for item in plan["L1"]] == rules.DICE_CELLS[2]
    assert [item["cell"] for item in plan["L2"]] == rules.DICE_CELLS[4]
    assert [item["cell"] for item in plan["L3"]] == rules.DICE_CELLS[6]

    # Case 2 (비대칭 8R + 4G): P2-04(desc), R=5, Y=3, G=4 -> spare_red=3, spare_green=0 (Y를 빨강 단일 모양 3개로 구성)
    st_asym = state.new_state()
    state.apply_result(
        st_asym,
        "READ_MISSION_INPUT",
        {},
        {"success": True, "barcode": "P2-04", "signal": list("RRRRRYYYGGGG"), "end_seen": True},
    )
    specs_asym = [("R", "box")] * 4 + [("R", "cylinder")] * 4 + [("G", "cylinder")] * 2 + [("G", "box")] * 2
    _load_all_12_pucks(st_asym, specs_asym)
    rec_asym = planner.build_allocation_record(st_asym)
    assert rec_asym["planned"]["valid"] is True
    assert rec_asym["planned"]["y_shape"] in ("cylinder", "box")
    assert len(rec_asym["planned"]["groups"]["Y"]) == 3

    # Case 3 (불가능 1 - 색상 부족): R=6인데 빨강 퍽이 5개뿐
    st_bad_color = state.new_state()
    state.apply_result(
        st_bad_color,
        "READ_MISSION_INPUT",
        {},
        {"success": True, "barcode": "P2-01", "signal": list("RRRRRRYYYGGG"), "end_seen": True},
    )
    _load_all_12_pucks(st_bad_color, [("R", "cylinder")] * 5 + [("G", "cylinder")] * 7)
    with pytest.raises(planner.PlanError):
        planner.assign(st_bad_color)

    # Case 4 (불가능 2 - Y 그룹 모양 통일 불가): R=4, Y=4, G=4인데 빨강은 전부 cylinder(6), 초록은 전부 box(6)
    st_bad_shape = state.new_state()
    state.apply_result(
        st_bad_shape,
        "READ_MISSION_INPUT",
        {},
        {"success": True, "barcode": "P2-01", "signal": list("RYGRYGRYGRYG"), "end_seen": True},
    )
    _load_all_12_pucks(st_bad_shape, [("R", "cylinder")] * 6 + [("G", "box")] * 6)
    with pytest.raises(planner.PlanError):
        planner.assign(st_bad_shape)


def test_full_lifecycle_guards_and_allocation_json():
    st = state.new_state()
    state.apply_result(
        st,
        "READ_MISSION_INPUT",
        {},
        {"success": True, "barcode": "P2-01", "signal": list("RGRRGRGGYGYG"), "end_seen": True},
    )
    specs = [
        ("R", "cylinder"), ("G", "cylinder"),
        ("R", "box"), ("G", "box"),
        ("R", "cylinder"), ("G", "cylinder"),
        ("R", "box"), ("G", "box"),
        ("R", "cylinder"), ("G", "cylinder"),
        ("R", "box"), ("G", "box"),
    ]
    _load_all_12_pucks(st, specs)
    assert st["phase"] == "UNLOAD"

    # 이미 CLEARED된 S1 재방문 차단 확인
    with pytest.raises(state.ToolError, match="REVISIT"):
        state.check_revisit(st, "GOTO", {"t": "S1"})

    plan = planner.assign(st)
    for loc in rules.LOCATIONS:
        state.apply_result(
            st,
            "GOTO",
            {"t": loc},
            {"success": True, "robot": {"here": loc, "pose": {"x": 3200, "y": 950, "yaw": 0}}},
        )
        items = plan[loc]
        # 일부만 놓으려 하면 PARTIAL_UNLOAD 거절
        with pytest.raises(state.ToolError, match="PARTIAL_UNLOAD"):
            state.check_unload(st, {"items": items[:-1]})
        # 잘못된 주사위 칸 번호면 WRONG_CELLS 거절
        bad_cell = 1 if items[0]["cell"] != 1 else 3
        with pytest.raises(state.ToolError, match="WRONG_CELLS"):
            state.check_unload(st, {"items": [{"puck_id": items[0]["puck_id"], "cell": bad_cell}, *items[1:]]})

        state.check_unload(st, {"items": items})
        unload_res = {
            "success": True,
            "stopped_at": None,
            "items": [
                {"puck_id": it["puck_id"], "cell": it["cell"], "ok": True, "offset_mm": [1.5, -1.0]}
                for it in items
            ],
        }
        assert contract.check_result("UNLOAD", {"items": items}, unload_res) == []
        state.apply_result(st, "UNLOAD", {"items": items}, unload_res)

        occ_map = {it["cell"]: it["puck_id"] for it in items}
        verify_res = {
            "success": True,
            "all_ok": True,
            "items": [{"puck_id": it["puck_id"], "cell": it["cell"], "ok": True, "seen_puck_id": it["puck_id"]} for it in items],
            "extra": [],
            "cells": [
                {
                    "cell": c,
                    "state": "occupied" if c in occ_map else "empty",
                    "puck_id": occ_map.get(c),
                }
                for c in range(rules.NUM_CELLS)
            ],
        }
        assert contract.check_result("VERIFY_LOCATION", {"items": items}, verify_res) == []
        state.apply_result(st, "VERIFY_LOCATION", {"items": items}, verify_res)

    alloc = planner.build_allocation_record(st)
    assert alloc["placed"]["consistency"]["complete"] is True
    assert alloc["placed"]["consistency"]["unique_count"] == 12
    assert alloc["placed"]["consistency"]["duplicate_ids"] == []
    assert alloc["placed"]["consistency"]["missing_ids"] == []
    assert alloc["placed"]["consistency"]["y_shape_unified"] is True

    # HOME 복귀 후 3초 미만 정지면 아직 GOTO 단계, 3초 이상 정지면 DONE 전환
    state.apply_result(
        st,
        "GOTO",
        {"t": "HOME"},
        {"success": True, "in_zone": True, "stopped_s": 1.5, "robot": {"here": "HOME", "pose": {"x": 734, "y": 950, "yaw": 180}}},
    )
    assert st["phase"] == "GOTO"
    state.apply_result(
        st,
        "GOTO",
        {"t": "HOME"},
        {"success": True, "in_zone": True, "stopped_s": 3.2, "robot": {"here": "HOME", "pose": {"x": 734, "y": 950, "yaw": 180}}},
    )
    assert st["phase"] == "DONE"


@pytest.mark.parametrize("yaw_deg", [0.0, 90.0, 180.0, -90.0])
def test_arducam_topview_cell_matching_across_headings(yaw_deg):
    yaw_rad = math.radians(yaw_deg)
    box_x, box_y = 0.285, -0.010
    dice6_cells = rules.DICE_CELLS[6]  # [0, 3, 6, 2, 5, 8]

    expected_loc = {}
    observed_xyz = []
    detected_classes = []
    observed_conf = []
    for idx, cell in enumerate(dice6_cells):
        mdx, mdy = _cell_map_offset(cell)
        bdx, bdy = _map_to_base_xy(mdx, mdy, yaw_rad)
        # 3mm 이내 미세 배치 오차 시뮬레이션
        observed_xyz.extend([box_x + bdx + 0.002, box_y + bdy - 0.001, 0.080])
        is_red = (idx % 2 == 0)
        detected_classes.append("red_cylinder" if is_red else "green_cylinder")
        observed_conf.append(0.96)
        expected_loc[cell] = {
            "puck_id": idx,
            "color": "R" if is_red else "G",
            "shape": "cylinder",
        }

    fake_pick_res = SimpleNamespace(
        observe_source="arducam",
        observed_box_xy=[box_x, box_y],
        observed_puck_xyz=observed_xyz,
        detected_classes=detected_classes,
        observed_puck_conf=observed_conf,
    )
    extracted = MissionBridge._extract_arducam_observation(fake_pick_res, 0.285, 0.0)
    assert extracted is not None
    ext_bx, ext_by, pucks = extracted
    assert len(pucks) == 6

    cells_out, cell_hits = MissionBridge._match_pucks_to_cells(
        ext_bx, ext_by, pucks, yaw_rad, expected_loc
    )
    occupied_cells = {obs.cell: obs.tag_id for obs in cells_out if obs.state == "occupied"}
    assert set(occupied_cells.keys()) == set(dice6_cells)
    assert set(occupied_cells.values()) == set(range(6))
    for cell in dice6_cells:
        assert math.hypot(*cell_hits[cell]["offset"]) < 0.005


def test_media_recorder_before_after_snapshots(tmp_path):
    import json
    from xsting_mission.commander.media_recorder import MediaRecorder

    class _DummyNode:
        def __init__(self):
            self.subs = []

        def create_subscription(self, msg_type, topic, cb, qos):
            self.subs.append((msg_type, topic, cb))
            return cb

    dummy = _DummyNode()
    rec = MediaRecorder(
        dummy,
        str(tmp_path),
        "/camera/color/image_raw/compressed",
        "/arducam/image_rect/compressed",
        video_fps=4.0,
    )
    # 가짜 JPEG 바이트 주입 후 before/after 스냅샷 기록 검증
    fake_jpeg = b"\xff\xd8\xff\xe0\x00\x10JFIF\x00\x01\x01\x00\x00\x01\x00\x01\x00\x00\xff\xd9"
    for _, topic, cb in dummy.subs:
        if topic.endswith("/compressed"):
            cb(SimpleNamespace(data=fake_jpeg))

    before = rec.capture_snapshot(
        1, "before", "UNLOAD", {}, {"here": "L1", "pose": {"x": 3200, "y": 950, "yaw": 0}}
    )
    after = rec.capture_snapshot(
        1, "after", "UNLOAD", {}, {"here": "L1", "pose": {"x": 3200, "y": 950, "yaw": 0}}
    )
    rec.close()

    assert len(before) == 2
    assert len(after) == 2
    for rel in before + after:
        assert (tmp_path / rel).is_file()
    index_data = json.loads((tmp_path / "media" / "media_index.json").read_text(encoding="utf-8"))
    assert len(index_data["snapshots"]) == 2
    assert index_data["snapshots"][0]["stage"] == "before"
    assert index_data["snapshots"][1]["stage"] == "after"

