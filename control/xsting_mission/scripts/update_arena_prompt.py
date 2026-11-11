#!/usr/bin/env python3
"""config/arena.yaml로 prompts/mission.md의 경기장 절을 다시 쓴다(--print는 출력만)."""
import argparse
import math
import re
import sys
from pathlib import Path

import yaml

PKG = Path(__file__).resolve().parents[1]
BEGIN, END = "<!-- arena:begin -->", "<!-- arena:end -->"
ROLE = {"HOME": "출발·복귀 구역", "S": "시작 박스", "L": "하차 박스"}
WALL_TABLE_HEADER = ["| ID | 방향 | 위치 | 범위 | 주변 |", "|---|---|---|---|---|"]


# ---------- 좁은 틈 계산 ----------

def seg_dist(point, start, end):
    """점과 선분 사이 거리."""
    (px, py), (ax, ay), (bx, by) = point, start, end
    dx, dy = bx - ax, by - ay
    ratio = ((px - ax) * dx + (py - ay) * dy) / max(dx * dx + dy * dy, 1e-9)
    ratio = max(0.0, min(1.0, ratio))
    return math.hypot(px - (ax + ratio * dx), py - (ay + ratio * dy))


def narrow_gaps(arena):
    """벽 끝과 다른 벽 사이 틈 중 로봇이 못 지나가는 것 [(벽, 끝, 상대, 폭)]."""
    width, height = arena["size"]
    border = {"바깥 벽": [((0, 0), (width, 0)), ((width, 0), (width, height)),
                       ((width, height), (0, height)),
                       ((0, height), (0, 0))]}
    all_walls = {**(arena.get("walls") or {}),
                 **(arena.get("virtual_walls") or {})}
    walls = {name: [(tuple(wall["from"]), tuple(wall["to"]))]
             for name, wall in all_walls.items()}
    limit = 2 * arena["costmap"]["padding"]

    gaps = []
    for name, [(start, end)] in walls.items():
        for tip in (start, end):
            dists = sorted((seg_dist(tip, *segment), other)
                           for other, segments in {**walls, **border}.items()
                           if other != name for segment in segments)
            dist, other = dists[0]
            if dist < 1:  # 다른 벽에 이어진 끝이라 틈이 없다
                continue
            free = dist - arena["wall_thickness"]  # 양쪽 벽 두께의 절반씩
            if free < limit:
                gaps.append((name, tip, other, free))
    return gaps


# ---------- 프롬프트 절 만들기 ----------

def wall_row(name, wall):
    (x1, y1), (x2, y2) = wall["from"], wall["to"]
    if x1 == x2:
        row = f"| {name} | 세로 | x={x1} | y {min(y1, y2)} ~ {max(y1, y2)} |"
    elif y1 == y2:
        row = f"| {name} | 가로 | y={y1} | x {min(x1, x2)} ~ {max(x1, x2)} |"
    else:
        row = f"| {name} | 대각 | ({x1}, {y1}) → ({x2}, {y2}) | |"
    return f"{row} {wall.get('note', '')} |"


def arena_section(arena):
    width, height = arena["size"]
    robot, places = arena["robot"], arena["places"]
    lines = [
        "## 좌표계",
        "- 원점: 경기장 왼쪽 아래 기둥 중심, 단위 mm. +X는 오른쪽, +Y는 지도 위쪽.",
        "- \"위/아래/왼쪽/오른쪽\"은 항상 지도 기준이다 (로봇이 바라보는 방향이 아님).",
        f"- 경기장은 바깥 벽 기둥 중심 기준 x 0~{width}, y 0~{height} "
        "(바깥 크기 4464×6364). 벽·기둥·박스 위치는 고정이다.",
        "", "## 장소",
        "| ID | 역할 | 위치 | 설명 |", "|---|---|---|---|"]
    for name, place in places.items():
        role = ROLE.get(name, ROLE.get(name[0], ""))
        x, y = place["target"][0], place["target"][1]
        lines.append(
            f"| {name} | {role} | ({x}, {y}) | {place.get('note', '')} |")

    lines += [
        "", "- 위치는 등록 위치다. 박스는 당일 약간 이동했을 수 있고, "
        "정확한 위치는 젯슨이 관측해 처리한다.",
        "", "## 벽",
        "- 바깥 벽이 경기장 테두리 전체를 막는다.",
        "- 안쪽 벽은 기둥 사이에 세운 벽판이다. 좌표는 양 끝 기둥 중심 (mm):",
        ""] + WALL_TABLE_HEADER
    lines += [wall_row(name, wall)
              for name, wall in (arena.get("walls") or {}).items()]
    virtual = arena.get("virtual_walls") or {}
    if virtual:
        lines += [
            "", "## 가벽",
            "- 실제 벽은 아니지만 이 미션에서 지나가지 않기로 한 선이다. "
            "벽과 똑같이 지나갈 수 없다.",
            ""] + WALL_TABLE_HEADER
        lines += [wall_row(name, wall) for name, wall in virtual.items()]

    lines += ["", "## 지나갈 수 없는 곳"]
    for obstacle in arena.get("obstacles") or []:
        (cx, cy), (sx, sy) = obstacle["center"], obstacle["size"]
        note = f" {obstacle['note']}" if obstacle.get("note") else ""
        lines.append(f"- {obstacle['name']}: {sx}×{sy}, 중심 ({cx}, {cy}), "
                     f"범위 x {cx - sx // 2}~{cx + sx // 2}, "
                     f"y {cy - sy // 2}~{cy + sy // 2}." + note)
    lines.append(
        f"- 시작 박스와 하차 박스 ({arena['box_size']}×{arena['box_size']}).")

    gaps = narrow_gaps(arena)
    xs = [point[0] for point in robot["footprint"]]
    ys = [point[1] for point in robot["footprint"]]
    lines.append(f"- 좁은 틈: 로봇이 크다 (좌우 {max(ys) - min(ys)} × "
                 f"앞뒤 {max(xs) - min(xs)}mm). 여유를 두어 "
                 f"폭 {2 * arena['costmap']['padding']}mm보다 좁은 틈은 "
                 "지나갈 수 없다:")
    for name, (x, y), other, free in gaps:
        lines.append(f"  - {name} 끝 ({x}, {y})과 {other} 사이 "
                     f"(틈 약 {int(free)}mm)")
    if not gaps:
        lines.append("  - 없음")
    lines.append("- 그 밖에 벽 끝과 다른 벽·바깥 벽 사이는 지나갈 수 있다. "
                 "세로 벽은 위·아래 끝을, 가로 벽은 왼쪽·오른쪽 끝을 돌아간다.")
    return "\n".join(lines)


# ---------- 실행 ----------

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--print", action="store_true",
                        help="mission.md를 고치지 않고 결과만 출력")
    options = parser.parse_args()

    arena = yaml.safe_load(
        (PKG / "config/arena.yaml").read_text(encoding="utf-8"))
    text = arena_section(arena)
    half_width = max(abs(point[1]) for point in arena["robot"]["footprint"])
    if arena["costmap"]["padding"] < half_width:
        print("경고: costmap.padding이 로봇 폭의 절반보다 작아 "
              "로봇이 벽에 닿는 경로도 나온다", file=sys.stderr)
    print(text)
    if options.print:
        return

    prompt = PKG / "prompts/mission.md"
    markdown = prompt.read_text(encoding="utf-8")
    if BEGIN not in markdown or END not in markdown:
        sys.exit(f"{prompt}에 {BEGIN} ... {END} 표시가 없음")
    markdown = re.sub(re.escape(BEGIN) + ".*?" + re.escape(END),
                      lambda _: f"{BEGIN}\n{text}\n{END}", markdown,
                      flags=re.S)
    prompt.write_text(markdown, encoding="utf-8")
    print(f"\n{prompt} 갱신")


if __name__ == "__main__":
    main()
