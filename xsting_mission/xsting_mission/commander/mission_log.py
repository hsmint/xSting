"""판 기록: mission.jsonl, JSON 파일 저장, decisions.json 병합, report.md."""
import json
import os
import threading
import time


def _pending_prefetch_index(records, record):
    """record로 바꿀 같은 turn의 미리 판단 카드 위치 (없거나 사람 명령이면 None)."""
    if record.get("decider") == "user":
        return None
    for index in range(len(records) - 1, -1, -1):
        prefetch = records[index].get("prefetch") or {}
        if (records[index].get("turn") == record.get("turn")
                and prefetch.get("status") in ("requested", "answered", "asking")):
            return index
    return None


class MissionLog:
    """한 판의 기록 폴더(run_<시각>)와 그 안의 파일들."""

    def __init__(self, log_dir, run_name, get_turn):
        self.run_id = time.strftime("%Y%m%d_%H%M%S")
        self.run_dir = os.path.join(log_dir, f"run_{self.run_id}")
        os.makedirs(self.run_dir, exist_ok=True)
        self._get_turn = get_turn  # 기록 시점의 turn (미리 판단 스레드도 쓴다)
        self._file = open(os.path.join(self.run_dir, "mission.jsonl"), "a",
                          encoding="utf-8")
        self._file_lock = threading.Lock()  # 미리 판단 스레드도 기록을 쓴다
        # 발행과 decisions.json 병합을 함께 묶는 잠금 (호출하는 쪽이 잡는다)
        self.records_lock = threading.Lock()
        self.records = []  # decisions.json

        run_name = " ".join(run_name.split())[:60]
        if run_name:
            name_path = os.path.join(self.run_dir, "name.txt")
            with open(name_path, "w", encoding="utf-8") as file:
                file.write(run_name)

    # ---------- 기록 파일 ----------
    def log(self, kind, data):
        line = json.dumps({"t": time.time(), "turn": self._get_turn(),
                           "kind": kind, "data": data},
                          ensure_ascii=False, default=str)
        with self._file_lock:
            self._file.write(line + "\n")
            self._file.flush()

    def save(self, filename, data):
        path = os.path.join(self.run_dir, filename)
        with open(path, "w", encoding="utf-8") as file:
            json.dump(data, file, ensure_ascii=False, indent=2)

    def add_decision(self, record):
        """판단 기록을 decisions.json에 넣는다. records_lock을 잡고 부른다."""
        # 결과(outcome)는 같은 turn의 판단에 합친다
        if "outcome" in record and record.get("decider") != "user":
            for decision in reversed(self.records):
                if (decision["turn"] == record["turn"]
                        and decision.get("decider") != "user"):
                    decision["outcome"] = record["outcome"]
                    break
        else:
            index = _pending_prefetch_index(self.records, record)
            if index is None:
                self.records.append(record)
            else:  # 미리 판단 카드를 실제 판단(또는 버림)으로 바꾼다
                self.records[index] = record
        self.save("decisions.json", self.records)

    # ---------- 사람이 읽는 보고서 ----------
    def write_report(self, state, model, effort, guard, halt):
        """판 전체를 사람이 읽기 쉬운 report.md로 남긴다."""
        records = self.records

        def total(*keys):  # 예: total("input_tokens_details", "cached_tokens")
            count = 0
            for record in records:
                value = record.get("usage") or {}
                for key in keys:
                    value = (value or {}).get(key)
                count += value or 0
            return count

        latencies = [record["latency_s"] for record in records
                     if record.get("latency_s")]
        world = state["world"]
        guard = "on" if guard else "off"
        halt = f", 중단: {halt}" if halt else ""
        average = sum(latencies) / max(len(latencies), 1)
        lines = [
            f"# 미션 실행 기록 {self.run_id}", "",
            f"- 모델: {model} (effort {effort}), 가드: {guard}",
            f"- 결과: phase={state['phase']}" + halt,
            f"- 판단 {len(records)}회, 응답 평균 {average:.1f}초 / "
            f"최대 {max(latencies, default=0):.1f}초",
            f"- 토큰: 입력 {total('input_tokens')} "
            f"(캐시 {total('input_tokens_details', 'cached_tokens')}), "
            f"출력 {total('output_tokens')} "
            f"(생각 {total('output_tokens_details', 'reasoning_tokens')})",
        ]
        if world["barcode"] and world["signal"]:
            counts = world["signal"]["counts"]
            lines.append(
                f"- 입력: 바코드 {world['barcode']['code']} "
                f"({world['barcode']['order']}), "
                f"신호 {''.join(world['signal']['sequence'])} "
                f"(R{counts['R']} Y{counts['Y']} G{counts['G']})")

        lines += ["", "## 하차 박스 검증", ""]
        for location, box in world["locations"].items():
            verified = box["verified"]
            placed = [f"{cell['puck_id']}→c{cell['cell']}"
                      for cell in box["cells"] if cell["state"] == "occupied"]
            if verified is None:
                check = "검증 안 함"
            elif (all(item.get("ok") for item in verified["items"])
                  and not verified["extra"]):
                check = "통과"
            else:
                check = f"불일치 (extra {verified['extra']})"
            lines.append(
                f"- {location}: {' '.join(placed) or '비어 있음'} — {check}")

        lines += ["", "## 판단 순서", "",
                  "| # | 누가 | 행동 | 인자 | 결과 | 응답(초) "
                  "| 생각 요약 / 응답 문장 |",
                  "|---|---|---|---|---|---|---|"]
        for record in records:
            outcome = record.get("outcome") or {}
            if record.get("error"):
                result = record["error"]
            elif outcome.get("success"):
                result = "✓"
            elif outcome:
                result = f"✗ {outcome.get('error')}"
            else:
                result = "-"
            note = " / ".join((record.get("reasoning") or [])
                              + (record.get("text") or []))
            note = note.replace("\n", " ").replace("|", "\\|")
            who = {"ai": "AI", "user": "사용자",
                   "rule": "규칙"}.get(record.get("decider"), "-")
            args = json.dumps(record.get("args", {}), ensure_ascii=False)
            lines.append(
                f"| {record['turn']} | {who} | {record.get('name') or '-'} "
                f"| `{args}` | {result} | {record.get('latency_s', '-')} "
                f"| {note[:300]} |")
        report_path = os.path.join(self.run_dir, "report.md")
        with open(report_path, "w", encoding="utf-8") as file:
            file.write("\n".join(lines) + "\n")
