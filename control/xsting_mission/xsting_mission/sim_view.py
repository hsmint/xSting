"""가짜 경기장을 브라우저로 보고 조작하는 작은 웹 서버 (파이썬 기본 라이브러리만 사용).

  GET  /            web/index.html
  GET  /map         고정 정보: 벽 격자, 박스·장소 좌표, 접근점
  GET  /decisions   중계 노드의 LLM 판단 기록 (보낸 상태, 생각 요약, 응답 원문, 토큰, 로봇 결과)
  GET  /state       현재 상태: 로봇 위치·경로, 신호등, 퍽, 적재함, 진행 중 명령, 명령 기록, 중계 노드 상태
  POST /command     {"name", "args"} 명령 보내기 (중계 노드와 같은 ROS action 경로)
  POST /time_scale  {"value"} 배속 바꾸기
  POST /reset       경기장을 시나리오 처음 상태로 (명령 실행 중이 아닐 때만)
  POST /control     {"cmd": "pause" | "resume" | "stop"} 일시정지·재개·미션 정지 (중계 노드에도 전달)
  GET  /runs        다시 실행할 수 있는 지난 판 목록 (~/xsting_logs/run_*)
  POST /replay      {"run_id"} 지난 판을 같은 문제로 맞추고 기록된 명령을 순서대로 다시 보낸다
  POST /view_run    {"run_id"} 다시 실행하지 않고 지난 판의 판단 기록을 LLM 판단 탭에 펼친다
"""
import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from .rules import NUM_CELLS, NUM_PUCKS, PLACES
from .tools import TOOL_NAMES


def check_args(name, args):
    """화면에서 온 명령을 도구 정의와 같은 형식인지 검사한다. 문제가 없으면 None."""
    if name not in TOOL_NAMES:
        return f"알 수 없는 행동: {name}"
    if not isinstance(args, dict):
        return "args는 객체여야 함"
    if name == "GOTO":
        return None if args.keys() == {"t"} and args["t"] in PLACES else f"GOTO 인자는 t ∈ {PLACES}"
    if name in ("UNLOAD", "VERIFY_LOCATION"):
        items = args.get("items")
        if args.keys() != {"items"} or not isinstance(items, list) or not items:
            return f"{name} 인자는 items 리스트 (1개 이상)"
        for it in items:
            if (not isinstance(it, dict) or it.keys() != {"puck_id", "cell"}
                    or it["puck_id"] not in range(NUM_PUCKS) or it["cell"] not in range(NUM_CELLS)):
                return f"items 항목은 {{puck_id: 0~{NUM_PUCKS - 1}, cell: 0~{NUM_CELLS - 1}}}"
        return None
    return None if not args else f"{name}은(는) 인자가 없음"


class ViewServer:
    def __init__(self, world, web_dir, port, logger, send_command, is_busy, control, reset, list_runs,
                 start_replay, view_run):
        index = Path(web_dir) / "index.html"

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                if self.path in ("/", "/index.html"):
                    self._send(index.read_bytes(), "text/html; charset=utf-8")
                elif self.path == "/map":
                    self._json(world.static_map())
                elif self.path == "/state":
                    self._json(world.snapshot())
                elif self.path == "/decisions":
                    self._json(world.decision_list())
                elif self.path == "/runs":
                    self._json(list_runs())
                else:
                    self.send_error(404)

            def do_POST(self):
                try:
                    body = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0))) or b"{}")
                except json.JSONDecodeError:
                    return self._json({"ok": False, "error": "JSON 형식 오류"}, 400)
                if self.path == "/command":
                    name, args = body.get("name"), body.get("args", {})
                    error = check_args(name, args)
                    if error:
                        return self._json({"ok": False, "error": error}, 400)
                    if is_busy():
                        return self._json({"ok": False, "error": "로봇이 다른 명령을 실행 중"}, 409)
                    logger.info(f"web command: {name} {json.dumps(args, ensure_ascii=False)}")
                    send_command(name, args)
                    self._json({"ok": True})
                elif self.path == "/time_scale":
                    try:
                        self._json({"ok": True, "value": world.set_time_scale(body["value"])})
                    except (KeyError, TypeError, ValueError):
                        self._json({"ok": False, "error": "value는 숫자"}, 400)
                elif self.path == "/control":
                    if body.get("cmd") not in ("pause", "resume", "stop"):
                        return self._json({"ok": False, "error": "cmd는 pause | resume | stop"}, 400)
                    control(body["cmd"])
                    self._json({"ok": True})
                elif self.path == "/replay":
                    if is_busy():
                        return self._json({"ok": False, "error": "명령 실행 중이거나 이미 다시 실행 중"}, 409)
                    try:
                        start_replay(str(body.get("run_id", "")))
                    except (FileNotFoundError, ValueError, KeyError) as e:
                        return self._json({"ok": False, "error": f"기록을 읽을 수 없음: {e}"}, 400)
                    self._json({"ok": True})
                elif self.path == "/view_run":
                    try:
                        view_run(str(body.get("run_id", "")))
                    except (FileNotFoundError, ValueError, KeyError) as e:
                        return self._json({"ok": False, "error": f"기록을 읽을 수 없음: {e}"}, 400)
                    self._json({"ok": True})
                elif self.path == "/reset":
                    if is_busy():
                        return self._json({"ok": False, "error": "명령 실행 중에는 초기화할 수 없음"}, 409)
                    reset()
                    logger.info("web: fake world reset")
                    self._json({"ok": True})
                else:
                    self.send_error(404)

            def _json(self, data, code=200):
                self._send(json.dumps(data, ensure_ascii=False).encode(), "application/json", code)

            def _send(self, body, content_type, code=200):
                self.send_response(code)
                self.send_header("Content-Type", content_type)
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def log_message(self, *args):  # 0.15초마다 오는 요청을 로그에 찍지 않는다
                pass

        self.httpd = ThreadingHTTPServer(("127.0.0.1", port), Handler)  # 이 노트북에서만 접속
        self.thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)
        self.thread.start()
        logger.info(f"sim view: http://localhost:{port}")

    def stop(self):
        self.httpd.shutdown()
