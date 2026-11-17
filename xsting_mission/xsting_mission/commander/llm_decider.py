"""ChatGPT(OpenAI Responses API) 판단: 상태를 보내고 다음 행동 하나를 받는다."""
import copy
import json
import time

import yaml

from ..core.state import to_message
from ..map.arena_map import ArenaMap
from .route_tool import CALC_TOOL, compare_routes
from .tools import ROBOT_TOOLS, TOOLS

# 안전 필터 오탐은 같은 입력을 다시 보내면 통과하는 경우가 많아 이만큼 더 보낸다
FLAGGED_RETRIES = 3
# 판단 한 번에 COMPARE_ROUTES를 부를 수 있는 횟수. 넘으면 로봇 행동만 보여 준다
MAX_CALC_CALLS = 3


class LlmUnavailable(Exception):
    """이번 판단에 LLM을 쓸 수 없음. 미션을 멈춘다."""


# ---------- 응답 해석 ----------
def _input_item(item):
    """응답 항목을 다음 요청의 입력 항목으로 (없는 값은 뺀다)."""
    return item.model_dump(exclude_none=True)


def _log_item(item):
    """기록용 응답 항목. 길고 읽을 수 없는 암호화된 생각은 뺀다."""
    data = item.model_dump()
    data.pop("encrypted_content", None)
    return data


def _sum_usage(usages):
    """여러 응답(계산 도구로 여러 번 요청한 판단)의 토큰 사용량을 더한다."""
    if not usages:
        return None

    def add(left, right):
        if isinstance(left, dict):
            return {key: add(left.get(key), right.get(key))
                    if isinstance(right, dict) else left.get(key)
                    for key in left}
        if isinstance(left, (int, float)) or isinstance(right, (int, float)):
            return (left or 0) + (right or 0)
        return left

    total = usages[0]
    for usage in usages[1:]:
        total = add(total, usage)
    return total


def _is_flagged(error):
    """OpenAI 안전 필터가 입력(프롬프트)을 걸러낸 400 오류인지."""
    message = str(error).lower()
    return "flagged" in message or "usage policy" in message


class LlmDecider:
    """상태 → (행동, 인자, 계획, 기록). 노드의 상태는 바꾸지 않는다."""

    def __init__(self, model, effort, reasoning_summary, sampling,
                 prompt_path, arena_path, logger, log, publish_decision):
        import openai  # LLM 판단에서만 쓰므로 여기서 불러온다
        self.openai = openai
        self.model = model
        self.effort = effort
        self.reasoning_summary = reasoning_summary
        self.sampling = sampling  # {"temperature", "top_p"} 중 보낼 것만
        self.logger = logger
        self.log = log  # MissionLog.log
        self.publish_decision = publish_decision  # 노드의 판단 카드 발행
        # 키: OPENAI_API_KEY
        self.client = openai.OpenAI(timeout=60.0, max_retries=2)
        with open(prompt_path, encoding="utf-8") as file:
            # 경기 중 바꾸지 않아야 도구 + 고정 프롬프트 앞부분이 캐시된다
            self.instructions = file.read()
        with open(arena_path, encoding="utf-8") as file:
            # 가짜 경기장과 같은 지도·코스트맵으로 COMPARE_ROUTES를 계산
            self.arena_map = ArenaMap(yaml.safe_load(file))

    # ---------- 요청 ----------
    def request(self, sent_state, extra=(), tools=TOOLS):
        """extra: 계산 도구를 쓴 뒤 다시 요청할 때 붙이는 앞 응답 항목과 계산 결과."""
        reasoning = {"effort": self.effort}
        if self.reasoning_summary:
            reasoning["summary"] = self.reasoning_summary
        text = ("현재 MissionState입니다. 규칙과 상태를 보고 다음 행동 하나를 "
                "도구로 호출하세요.\n" + to_message(sent_state))
        options = dict(self.sampling)
        if self.effort != "none":
            # store=False에서 계산 결과로 이어 판단하려면 앞 생각(암호화)을 돌려줘야 한다
            options["include"] = ["reasoning.encrypted_content"]
        return self.client.responses.create(
            model=self.model,
            instructions=self.instructions,
            input=[{"role": "user", "content": text}, *extra],
            tools=list(tools),
            tool_choice="auto",
            parallel_tool_calls=False,  # 한 번에 도구 하나
            reasoning=reasoning,
            max_output_tokens=16000,
            prompt_cache_key="xsting_mission",
            store=False,  # 매번 상태만 보내므로 대화를 서버에 남기지 않는다
            **options,
        )

    def _request_with_retries(self, sent_state, extra, tools, started,
                              quiet=False):
        """요약 막힘·안전 필터 오탐이면 다시 보내는 요청 (quiet면 실패 기록을 안 남김)."""
        openai = self.openai
        try:
            flagged_retries = FLAGGED_RETRIES
            request_state = sent_state
            while True:
                try:
                    return self.request(request_state, extra, tools)
                except openai.BadRequestError as error:
                    # 생각 요약이 계정·모델에 따라 막혀 있으면 요약 없이 다시 요청한다
                    if (self.reasoning_summary
                            and "summar" in str(error).lower()):
                        self.logger.warning(
                            "reasoning summary unavailable, retrying "
                            f"without it: {error}")
                        self.reasoning_summary = None
                    # 오탐은 지난 계획의 근거(reason) 탓일 수 있어 그것만 빼고 다시 보낸다
                    elif _is_flagged(error) and flagged_retries > 0:
                        attempt = FLAGGED_RETRIES - flagged_retries + 1
                        self.logger.warning(
                            "prompt flagged, retrying without plan text "
                            f"({attempt}/{FLAGGED_RETRIES}): {error}")
                        self.log("flagged", {"error": repr(error)})
                        flagged_retries -= 1
                        request_state = copy.deepcopy(sent_state)
                        plan = request_state["memory"]["plan"]
                        if plan:
                            plan.pop("reason", None)
                    else:
                        raise
        except openai.BadRequestError as error:
            # 몇 번을 다시 보내도 걸리면 노드를 죽이지 않고 기록을 남긴 뒤 멈춘다
            if _is_flagged(error):
                if not quiet:
                    self.publish_decision({
                        "decider": "ai", "input_state": sent_state,
                        "error": f"PROMPT_FLAGGED {error!r}",
                        "latency_s": round(time.monotonic() - started, 2)})
                raise LlmUnavailable(f"PROMPT_FLAGGED {error!r}")
            raise  # 요청 형식 오류는 코드나 설정 문제라 숨기지 않는다
        except (openai.AuthenticationError, openai.NotFoundError):
            raise  # 키·모델 이름 오류는 설정 문제라 숨기지 않는다
        except (openai.RateLimitError, openai.APIConnectionError,
                openai.APIStatusError) as error:
            if not quiet:
                self.publish_decision({
                    "decider": "ai", "input_state": sent_state,
                    "error": f"API_ERROR {error!r}",
                    "latency_s": round(time.monotonic() - started, 2)})
            raise LlmUnavailable(f"API_ERROR {error!r}")

    # ---------- 판단 ----------
    def llm_decide(self, sent_state, quiet=False):
        """(행동, 인자, 계획, 기록)을 받는다. 상태는 안 바꾸고, 못 쓰면 LlmUnavailable."""
        started = time.monotonic()
        extra, calcs, responses = [], [], []
        while True:
            # 계산 도구를 MAX_CALC_CALLS번 쓰면 로봇 행동만 보여 줘서 고르게 한다
            tools = TOOLS if len(calcs) < MAX_CALC_CALLS else ROBOT_TOOLS
            response = self._request_with_retries(
                sent_state, extra, tools, started, quiet)
            responses.append(response)
            call = next((item for item in response.output
                         if item.type == "function_call"), None)
            if (call is None or call.name != CALC_TOOL
                    or response.status != "completed"
                    or len(calcs) >= MAX_CALC_CALLS):
                break
            try:
                calc_args = json.loads(call.arguments)
            except json.JSONDecodeError:
                calc_args = {}
            result = compare_routes(self.arena_map, calc_args, sent_state)
            calcs.append({"routes": calc_args.get("routes"),
                          "result": result})
            self.log("calc", calcs[-1])
            # 앞 응답(생각·호출)과 계산 결과를 붙여 같은 판단을 이어서 요청한다
            extra += [_input_item(item) for item in response.output]
            extra.append({"type": "function_call_output",
                          "call_id": call.call_id,
                          "output": json.dumps(result, ensure_ascii=False)})

        latency = time.monotonic() - started
        texts, refusals, reasoning = [], [], []
        for each_response in responses:
            for item in each_response.output:
                if item.type == "reasoning":
                    reasoning += [part.text for part in (item.summary or [])
                                  if getattr(part, "text", None)]
                elif item.type == "message":
                    for part in item.content:
                        if part.type == "output_text":
                            texts.append(part.text)
                        elif part.type == "refusal":
                            refusals.append(part.refusal)
        usage = _sum_usage([each_response.usage.model_dump()
                            for each_response in responses
                            if each_response.usage])
        record = {
            "input_state": sent_state, "latency_s": round(latency, 2),
            "status": response.status, "reasoning": reasoning, "text": texts,
            "refusal": refusals, "usage": usage,
            "raw_output": [_log_item(item) for each_response in responses
                           for item in each_response.output],
            "calcs": calcs, "rounds": len(responses),
            "name": call.name if call else None,
            "arguments": call.arguments if call else None}
        self.log("llm", {key: value for key, value in record.items()
                         if key != "input_state"})

        error = None
        if response.status != "completed":  # incomplete: max_output_tokens 초과 등
            error = f"STATUS_{response.status}"
        elif refusals:
            error = f"REFUSAL {refusals}"
        elif call is None:
            error = f"NO_TOOL_CALL {texts}"
        elif call.name == CALC_TOOL:  # 계산 도구만 부르고 로봇 행동을 고르지 않음
            error = "TOO_MANY_CALC_CALLS"
        else:
            try:
                args = json.loads(call.arguments)
            except json.JSONDecodeError:
                error = f"BAD_ARGUMENTS {call.arguments!r}"
        if error:
            if not quiet:
                self.publish_decision({**record, "decider": "ai",
                                       "error": error})
            raise LlmUnavailable(error)

        # 계획은 상태에 남기고, 로봇에는 행동 인자만 보낸다
        plan = args.pop("plan", None)
        return call.name, args, plan, record
