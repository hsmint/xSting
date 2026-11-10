#!/usr/bin/env python3
"""config/arena.yaml로 고정 프롬프트의 "3. ARENA (경기장)" 절을 다시 쓴다.

prompts/mission.md의 <!-- arena:begin --> ~ <!-- arena:end --> 사이를 아래 내용으로 바꾼다:
  - 좌표계, 장소 표
  - 바깥 벽, 안쪽 벽 표, 가벽 표(있을 때), 장애물
  - 로봇 크기와, 로봇이 지나갈 수 없어 막힌 좁은 틈 (벽 끝과 다른 벽 사이가 2 × costmap.padding보다 좁은 곳)
  - 경로 길이 구하는 법: 로봇을 점으로 보고 벽·박스를 costmap.padding만큼 부풀려 A*로 찾는 방법 (값은 LLM이 계산한다)

  python3 scripts/update_arena_prompt.py            # mission.md를 고친다
  python3 scripts/update_arena_prompt.py --print    # 파일은 그대로 두고 결과만 출력

arena.yaml을 고치면 다시 실행하고, llm_commander를 다시 켠다.
"""
import argparse
import math
import re
import sys
from pathlib import Path

import yaml

PKG = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG))

PLACES = ("HOME", "S1", "S2", "S3", "S4", "S5", "S6", "L1", "L2", "L3")
BEGIN, END = "<!-- arena:begin -->", "<!-- arena:end -->"


def seg_dist(p, a, b):
    """점 p와 선분 ab 사이 거리."""
    (px, py), (ax, ay), (bx, by) = p, a, b
    dx, dy = bx - ax, by - ay
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / max(dx * dx + dy * dy, 1e-9)))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def narrow_gaps(arena):
    """벽 끝에서 다른 벽(바깥 벽 포함)까지의 틈 중 로봇이 못 지나가는 것. [(벽 ID, 끝 좌표, 상대, 틈 폭)]"""
    w, h = arena["size"]
    border = {"바깥 벽": [((0, 0), (w, 0)), ((w, 0), (w, h)), ((w, h), (0, h)), ((0, h), (0, 0))]}
    walls = {k: [(tuple(v["from"]), tuple(v["to"]))]
             for k, v in {**(arena.get("walls") or {}), **(arena.get("virtual_walls") or {})}.items()}
    limit = 2 * arena["costmap"]["padding"]
    gaps = []
    for name, [(a, b)] in walls.items():
        for end in (a, b):
            dists = sorted((seg_dist(end, *s), other) for other, segs in {**walls, **border}.items()
                           if other != name for s in segs)
            d, other = dists[0]
            if d < 1:  # 다른 벽에 이어진 끝이라 틈이 없다
                continue
            free = d - arena["wall_thickness"]  # 양쪽 벽 두께의 절반씩
            if free < limit:
                gaps.append((name, end, other, free))
    return gaps


ROLE = {"HOME": "출발·복귀 구역", "S": "시작 박스", "L": "하차 박스"}


def wall_row(k, v):
    (x1, y1), (x2, y2) = v["from"], v["to"]
    if x1 == x2:
        row = f"| {k} | 세로 | x={x1} | y {min(y1, y2)} ~ {max(y1, y2)} |"
    elif y1 == y2:
        row = f"| {k} | 가로 | y={y1} | x {min(x1, x2)} ~ {max(x1, x2)} |"
    else:
        row = f"| {k} | 대각 | ({x1}, {y1}) → ({x2}, {y2}) | |"
    return f"{row} {v.get('note', '')} |"


def arena_section(arena):
    w, h = arena["size"]
    rb, places = arena["robot"], arena["places"]
    out = ["## 좌표계",
           "- 원점: 경기장 왼쪽 아래 기둥 중심, 단위 mm. +X는 오른쪽, +Y는 지도 위쪽.",
           "- \"위/아래/왼쪽/오른쪽\"은 항상 지도 기준이다 (로봇이 바라보는 방향이 아님).",
           f"- 경기장은 바깥 벽 기둥 중심 기준 x 0~{w}, y 0~{h} (바깥 크기 4464×6364). 벽·기둥·박스 위치는 고정이다.",
           "", "## 장소",
           "| ID | 역할 | 위치 | 설명 |", "|---|---|---|---|"]
    for name, p in places.items():
        role = ROLE.get(name, ROLE.get(name[0], ""))
        out.append(f"| {name} | {role} | ({p['target'][0]}, {p['target'][1]}) | {p.get('note', '')} |")
    out += ["", "- 위치는 등록 위치다. 박스는 당일 약간 이동했을 수 있고, 정확한 위치는 젯슨이 관측해 처리한다.",
            "", "## 벽",
            "- 바깥 벽이 경기장 테두리 전체를 막는다.",
            "- 안쪽 벽은 기둥 사이에 세운 벽판이다. 좌표는 양 끝 기둥 중심 (mm):", "",
            "| ID | 방향 | 위치 | 범위 | 주변 |", "|---|---|---|---|---|"]
    out += [wall_row(k, v) for k, v in (arena.get("walls") or {}).items()]
    virtual = arena.get("virtual_walls") or {}
    if virtual:
        out += ["", "## 가벽",
                "- 실제 벽은 아니지만 이 미션에서 지나가지 않기로 한 선이다. 벽과 똑같이 지나갈 수 없다.", "",
                "| ID | 방향 | 위치 | 범위 | 주변 |", "|---|---|---|---|---|"]
        out += [wall_row(k, v) for k, v in virtual.items()]
    out += ["", "## 지나갈 수 없는 곳"]
    for o in arena.get("obstacles") or []:
        (cx, cy), (sx, sy) = o["center"], o["size"]
        out.append(f"- {o['name']}: {sx}×{sy}, 중심 ({cx}, {cy}), 범위 x {cx - sx // 2}~{cx + sx // 2}, "
                   f"y {cy - sy // 2}~{cy + sy // 2}." + (f" {o['note']}" if o.get("note") else ""))
    out.append(f"- 시작 박스와 하차 박스 ({arena['box_size']}×{arena['box_size']}).")
    gaps = narrow_gaps(arena)
    xs, ys = [p[0] for p in rb["footprint"]], [p[1] for p in rb["footprint"]]
    out.append(f"- 좁은 틈: 로봇이 크다 (좌우 {max(ys) - min(ys)} × 앞뒤 {max(xs) - min(xs)}mm). 여유를 두어 "
               f"폭 {2 * arena['costmap']['padding']}mm보다 좁은 틈은 지나갈 수 없다:")
    for name, (x, y), other, free in gaps:
        out.append(f"  - {name} 끝 ({x}, {y})과 {other} 사이 (틈 약 {int(free)}mm)")
    if not gaps:
        out.append("  - 없음")
    pad, half = arena["costmap"]["padding"], arena["wall_thickness"] / 2
    out += ["- 그 밖에 벽 끝과 다른 벽·바깥 벽 사이는 지나갈 수 있다. 세로 벽은 위·아래 끝을, 가로 벽은 왼쪽·오른쪽 끝을 돌아간다.",
            "", "## 경로 길이 구하는 법 (A*)",
            "- 거리표는 없다. 로봇 위치(`robot.pose`)와 목표 위치(장소 표)로 경로를 직접 찾아 길이를 구한다.",
            f"- 로봇은 점으로 본다. 대신 벽·박스를 {pad}mm씩 부풀린다: 로봇 중심은 벽 중심선에서 {pad + half:.0f}mm "
            f"(패딩 {pad} + 벽 두께 절반 {half:.0f}), 박스 가장자리에서 {pad}mm보다 가까이 갈 수 없다.",
            "- A*처럼 찾는다:",
            "  1. 출발점에서 목표까지 직선이 부풀린 벽·박스를 지나지 않으면 그 직선이 경로다.",
            "  2. 막히면 가로막는 벽의 끝(기둥)을 돌아간다. 돌아가는 점은 벽 끝에서 바깥으로 "
            f"{pad + half:.0f}mm 떨어진 곳이다. 좁은 틈(위 목록)으로는 돌아갈 수 없다.",
            "  3. 돌아갈 끝이 여러 개면 각각의 길이(지금까지 간 거리 + 남은 직선거리)를 비교해 짧은 쪽부터 따진다.",
            "  4. 경로 길이 = 꺾이는 점 사이 직선거리의 합. 직선거리 = √(Δx² + Δy²).",
            "- 목표는 대상 중심으로 잡는다 (로봇은 대상 둘레 330~520mm에 서므로 실제로는 조금 짧다).",
            "- 실제 경로 계산과 주행은 젯슨이 한다. 너는 목적지 ID만 정하고, 순서를 정하려고 길이를 계산한다."]
    return "\n".join(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--print", action="store_true", help="mission.md를 고치지 않고 결과만 출력")
    opt = ap.parse_args()

    arena = yaml.safe_load((PKG / "config/arena.yaml").read_text(encoding="utf-8"))
    text = arena_section(arena)
    if arena["costmap"]["padding"] < max(abs(p[1]) for p in arena["robot"]["footprint"]):
        print("경고: costmap.padding이 로봇 폭의 절반보다 작아 로봇이 벽에 닿는 경로도 나온다", file=sys.stderr)
    print(text)
    if opt.print:
        return
    prompt = PKG / "prompts/mission.md"
    md = prompt.read_text(encoding="utf-8")
    if BEGIN not in md or END not in md:
        sys.exit(f"{prompt}에 {BEGIN} ... {END} 표시가 없음")
    md = re.sub(re.escape(BEGIN) + ".*?" + re.escape(END), lambda _: f"{BEGIN}\n{text}\n{END}", md, flags=re.S)
    prompt.write_text(md, encoding="utf-8")
    print(f"\n{prompt} 갱신")


if __name__ == "__main__":
    main()
