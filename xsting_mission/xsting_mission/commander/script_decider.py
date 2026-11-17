"""고정 행동 목록(config/fixed_actions.yaml)으로 판단한다 (AI를 쓰지 않는다)."""
import yaml

from ..core import planner
from .llm_decider import LlmUnavailable
from .tools import TOOL_NAMES


class ScriptDecider:
    """고정 행동 목록을 차례로 내놓고, 실패하면 retries번까지 다시 보낸다."""

    def __init__(self, path):
        self.load_script(path)

    def load_script(self, path):
        """고정 행동 목록을 [(이름, 인자)]로 읽는다 (형식은 fixed_actions.yaml 주석)."""
        with open(path, encoding="utf-8") as file:
            script = yaml.safe_load(file)
        self.actions, self.position = [], 0
        self.retries = script.get("retries", 0)
        self.stop_on_fail = script.get("stop_on_fail", False)
        for item in script["actions"]:
            if isinstance(item, str):
                name, args = item, {}
            else:
                (name, value), = item.items()
                # UNLOAD·VERIFY_LOCATION auto: 그 자리에서 planner로 채움
                if value == "auto":
                    args = {"items": "auto"}
                elif isinstance(value, str):
                    args = {"t": value}
                elif isinstance(value, list):
                    args = {"items": value}
                else:
                    args = value
            if name not in TOOL_NAMES:
                raise ValueError(f"{path}: 모르는 행동 {name}")
            self.actions.append((name, args))
        self.fails = 0  # 지금 행동의 연속 실패 수

    def decide_script(self, state):
        """다음 (행동, 인자, 계획, 설명). 상태는 안 바꾼다."""
        last = state["last_action"]
        last_failed = (self.position > 0 and last is not None
                       and not last["success"])
        if last_failed and self.fails < self.retries:
            self.fails += 1
            name, args = self.actions[self.position - 1]
            note = (f"고정 행동 {self.position}/{len(self.actions)} 다시 시도 "
                    f"({self.fails}/{self.retries})")
        else:
            if last_failed and self.stop_on_fail:
                raise LlmUnavailable(
                    f"SCRIPT_FAILED {last['name']} {last['error']}")
            if self.position >= len(self.actions):
                raise LlmUnavailable("SCRIPT_END")
            name, args = self.actions[self.position]
            self.position += 1
            self.fails = 0
            note = f"고정 행동 {self.position}/{len(self.actions)}"
        if args.get("items") == "auto":
            args = {"items": self.planned_items(state, name)}
            note += " (할당 계획으로 items 채움)"

        # 고정 행동 목록의 남은 순서를 계획으로 보여 준다 (화면 표시용)
        rest = []
        for step_name, step_args in self.actions[self.position:]:
            if "items" not in step_args:
                rest.append(f"{step_name} {step_args.get('t', '')}".strip())
            elif step_args["items"] == "auto":
                rest.append(f"{step_name} auto")
            else:
                rest.append(f"{step_name} …")
        plan = {"assignment": "UNLOAD·VERIFY_LOCATION auto는 할당 계획(planner.py)"
                              "으로 채움",
                "route": rest,
                "reason": "고정 행동 목록 (config/fixed_actions.yaml)",
                "change": ""}
        return name, args, plan, note

    @staticmethod
    def planned_items(state, name):
        """지금 하차 박스에서 UNLOAD·VERIFY_LOCATION할 items를 할당 계획에서 꺼낸다."""
        location = state["robot"]["here"]
        try:
            items = planner.assign(state).get(location)
        except planner.PlanError as error:
            raise LlmUnavailable(f"PLAN_FAILED {error}")
        if items is None:
            raise LlmUnavailable(f"PLAN_FAILED {name} auto는 하차 박스에서만 "
                                 f"(지금 here={location})")
        if name == "UNLOAD":  # 이미 놓은 퍽은 빼고, 적재함에 있는 것만
            pucks = state["world"]["pucks"]
            items = [item for item in items
                     if pucks[item["puck_id"]]["where"] == "storage"]
        return items
