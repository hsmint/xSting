"""가짜 경기장을 브라우저로 보고 조작하는 작은 웹 서버 (표준 라이브러리만 사용)."""
import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from ..core import contract


def check_args(name, args):
    """화면 명령이 contract.py 형식과 맞는지 검사한다 (맞으면 None)."""
    errors = contract.check_args(name, args)
    return "; ".join(errors) if errors else None


class ViewServer:
    def __init__(self, world, web_dir, port, logger, send_command, is_busy,
                 control, reset, list_runs, start_replay, view_run, name_run,
                 delete_run):
        index = Path(web_dir) / "index.html"

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                if self.path in ("/", "/index.html"):
                    self._send(index.read_bytes(), "text/html; charset=utf-8")
                elif self.path == "/map":  # 벽 격자, 박스·장소 좌표, 접근점
                    self._json(world.static_map())
                elif self.path == "/state":
                    self._json(world.snapshot())
                elif self.path == "/decisions":  # 판단 기록
                    self._json(world.decision_list())
                elif self.path == "/runs":  # 다시 실행할 수 있는 지난 판 목록
                    self._json(list_runs())
                else:
                    self.send_error(404)

            def do_POST(self):
                try:
                    length = int(self.headers.get("Content-Length", 0))
                    body = json.loads(self.rfile.read(length) or b"{}")
                except json.JSONDecodeError:
                    return self._json(
                        {"ok": False, "error": "JSON 형식 오류"}, 400)

                # 중계 노드와 같은 ROS action 경로로 명령 보내기
                if self.path == "/command":
                    name, args = body.get("name"), body.get("args", {})
                    error = check_args(name, args)
                    if error:
                        return self._json({"ok": False, "error": error}, 400)
                    if is_busy():
                        return self._json(
                            {"ok": False, "error": "로봇이 다른 명령을 실행 중"},
                            409)
                    logger.info(f"web command: {name} "
                                f"{json.dumps(args, ensure_ascii=False)}")
                    send_command(name, args)
                    self._json({"ok": True})
                elif self.path == "/time_scale":
                    try:
                        self._json({"ok": True, "value":
                                    world.set_time_scale(body["value"])})
                    except (KeyError, TypeError, ValueError):
                        self._json({"ok": False, "error": "value는 숫자"}, 400)
                # 일시정지·재개·미션 정지 (중계 노드에도 전달)
                elif self.path == "/control":
                    if body.get("cmd") not in ("pause", "resume", "stop"):
                        return self._json(
                            {"ok": False,
                             "error": "cmd는 pause | resume | stop"}, 400)
                    control(body["cmd"])
                    self._json({"ok": True})

                # ---------- 지난 판 ----------
                elif self.path == "/replay":
                    if is_busy():
                        return self._json(
                            {"ok": False,
                             "error": "명령 실행 중이거나 이미 다시 실행 중"},
                            409)
                    try:
                        start_replay(str(body.get("run_id", "")))
                    except (FileNotFoundError, ValueError, KeyError) as error:
                        return self._json(
                            {"ok": False,
                             "error": f"기록을 읽을 수 없음: {error}"}, 400)
                    self._json({"ok": True})
                elif self.path == "/view_run":
                    try:
                        view_run(str(body.get("run_id", "")))
                    except (FileNotFoundError, ValueError, KeyError) as error:
                        return self._json(
                            {"ok": False,
                             "error": f"기록을 읽을 수 없음: {error}"}, 400)
                    self._json({"ok": True})
                elif self.path == "/run_name":  # 빈 이름이면 이름을 지운다
                    try:
                        name = name_run(str(body.get("run_id", "")),
                                        str(body.get("name", "")))
                    except FileNotFoundError as error:
                        return self._json(
                            {"ok": False, "error": f"기록이 없음: {error}"},
                            400)
                    self._json({"ok": True, "name": name})
                elif self.path == "/run_delete":
                    try:
                        delete_run(str(body.get("run_id", "")))
                    except FileNotFoundError as error:
                        return self._json(
                            {"ok": False, "error": f"기록이 없음: {error}"},
                            400)
                    except ValueError as error:
                        return self._json(
                            {"ok": False, "error": str(error)}, 409)
                    logger.info(f"web: deleted run {body.get('run_id')}")
                    self._json({"ok": True})

                # ---------- 실패 주입·초기화 ----------
                elif self.path == "/fault":
                    try:
                        fault = world.add_fault(body)
                    except (ValueError, TypeError) as error:
                        return self._json(
                            {"ok": False, "error": str(error)}, 400)
                    logger.info(f"web: fault {fault}")
                    self._json({"ok": True, "fault": fault})
                elif self.path == "/perception":
                    try:
                        perception = world.set_perception(
                            body["fail_rate"], body["search_success_rate"])
                    except (KeyError, TypeError, ValueError):
                        return self._json(
                            {"ok": False, "error": "fail_rate, "
                             "search_success_rate는 0~100 숫자"}, 400)
                    logger.info(f"web: perception {perception}")
                    self._json({"ok": True, "perception": perception})
                elif self.path == "/fault_remove":
                    world.remove_fault(body.get("id"))
                    self._json({"ok": True})
                elif self.path == "/reset":
                    if is_busy():
                        return self._json(
                            {"ok": False,
                             "error": "명령 실행 중에는 초기화할 수 없음"},
                            409)
                    reset()
                    logger.info("web: fake world reset")
                    self._json({"ok": True})
                else:
                    self.send_error(404)

            def _json(self, data, code=200):
                self._send(json.dumps(data, ensure_ascii=False).encode(),
                           "application/json", code)

            def _send(self, body, content_type, code=200):
                self.send_response(code)
                self.send_header("Content-Type", content_type)
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def log_message(self, *args):  # 0.15초마다 오는 요청은 찍지 않는다
                pass

        # 이 노트북에서만 접속
        self.httpd = ThreadingHTTPServer(("127.0.0.1", port), Handler)
        self.thread = threading.Thread(target=self.httpd.serve_forever,
                                       daemon=True)
        self.thread.start()
        logger.info(f"sim view: http://localhost:{port}")

    def stop(self):
        self.httpd.shutdown()
