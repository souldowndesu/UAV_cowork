# -*- coding: utf-8 -*-
"""可视化 Web 服务器（标准库，无外部依赖）。

数据源（可同时提供多个）：
- ``recorder``：运行中的 Recorder → 实时模式（浏览器轮询 /api/live）。
- ``library``：RecordingLibrary → 回放库模式（扫描 data/records，页面下拉选择）。
- ``recording``：单个录制 dict → 单文件回放（向后兼容）。

端点：
- ``GET /``                  → 三维回放页面（viewer.html）
- ``GET /api/status``        → 数据源/模式信息
- ``GET /api/live``          → 最新一帧（实时）
- ``GET /api/recordings``    → 录制列表（含元信息，供下拉选择）
- ``GET /api/recording?id=N``→ 第 N 条录制的完整数据（回放）
"""
from __future__ import annotations

import json
import os
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

_VIEWER_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            "web", "viewer.html")


class _Handler(BaseHTTPRequestHandler):
    server_ref = None  # 由 VizServer.start 注入

    def _send(self, code, body, ctype="application/json; charset=utf-8"):
        if isinstance(body, (dict, list)):
            body = json.dumps(body, ensure_ascii=False)
        data = body.encode("utf-8") if isinstance(body, str) else body
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Access-Control-Allow-Origin", "*")
        # 禁用缓存，避免浏览器用旧版页面/旧数据
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
        self.send_header("Pragma", "no-cache")
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        try:
            self._get()
        except (OSError, ValueError, KeyError, TypeError, EOFError) as exc:
            self._send(422, {"error": "无法读取录制：" + str(exc)})

    def _get(self):
        parsed = urlparse(self.path)
        path = parsed.path
        qs = parse_qs(parsed.query)
        srv = self.server_ref

        if path in ("/", "/index.html", "/viewer.html"):
            try:
                with open(_VIEWER_PATH, "r", encoding="utf-8") as f:
                    self._send(200, f.read(), "text/html; charset=utf-8")
            except FileNotFoundError:
                self._send(404, "viewer.html 不存在")
            return

        if path == "/api/status":
            self._send(200, {
                "live": srv.recorder is not None,
                "replay": srv.recording is not None,
                "library": srv.library is not None,
                "recordings": srv.library.count() if srv.library is not None else 0,
                "simulator": srv.simulator is not None,
                "simulator_default": srv.simulator is not None and srv.simulator_default,
            })
            return

        if path == '/api/simulator':
            self._send(200, dict(available=True,**srv.simulator.status()) if srv.simulator else {'available':False})
            return

        if path == '/api/simulator/frame.jpg':
            frame=srv.simulator.image() if srv.simulator else None
            self._send(200,frame,'image/jpeg') if frame else self._send(503,{'error':'模拟器画面尚未就绪'})
            return

        if path == "/api/live":
            if srv.recorder is None:
                self._send(200, {"empty": True})
            else:
                self._send(200, srv.recorder.snapshot_live() or {"empty": True})
            return

        if path == "/api/recordings":
            if srv.library is None:
                self._send(200, [])
            else:
                self._send(200, srv.library.list())
            return

        if path == "/api/recording":
            if srv.library is not None:
                raw = qs.get("id", [None])[0]
                try:
                    # A 16-character stable hash may consist entirely of digits.
                    idx = int(raw) if raw is not None and raw.isdigit() and len(raw)<16 else raw
                except (TypeError, ValueError):
                    idx = None
                data = srv.library.get(idx) if idx is not None else None
                if data is None:
                    self._send(200, {"empty": True})
                else:
                    self._send(200, data)
                return
            if srv.recording is None:
                self._send(200, {"empty": True})
            else:
                self._send(200, srv.recording)
            return

        self._send(404, {"error": "not found"})

    def log_message(self, *args):
        pass  # 静默，避免刷屏

    def do_POST(self):
        if urlparse(self.path).path != '/api/simulator':
            self._send(404,{'error':'not found'});return
        if self.server_ref.simulator is None:
            self._send(409,{'error':'请运行 replay.py 启用场景回放，不要使用 --no-simulator'});return
        origin=self.headers.get('Origin')
        if origin:
            source=urlparse(origin)
            if source.scheme!='http' or source.hostname not in ('localhost','127.0.0.1') or source.port!=self.server.server_port:
                self._send(403,{'error':'场景控制请求必须来自本机回放页面'});return
        if self.headers.get('Content-Type','').split(';')[0]!='application/json':
            self._send(415,{'error':'Expected application/json'});return
        try:
            length=int(self.headers.get('Content-Length','0'))
            if not 0<length<=4096:raise ValueError('Invalid request size')
            request=json.loads(self.rfile.read(length))
            status=self.server_ref.simulator.submit(request)
            self._send(200,dict(available=True,**status))
        except (ValueError,TypeError) as exc:
            self._send(422,{'error':str(exc)})


class VizServer:
    def __init__(self, recorder=None, recording=None, library=None, port: int = 8765, simulator=None,
                 simulator_default=False):
        self.recorder = recorder
        self.recording = recording
        self.library = library
        self.simulator = simulator
        self.simulator_default = bool(simulator_default)
        self.port = int(port)
        self._httpd = None
        self._thread = None

    @property
    def url(self):
        return f"http://127.0.0.1:{self.port}/"

    def start(self):
        if self._httpd is not None:
            return
        handler=type("BoundHandler",(_Handler,),{"server_ref":self})
        self._httpd = ThreadingHTTPServer(("127.0.0.1", self.port), handler)
        self.port = self._httpd.server_address[1]
        self._thread = threading.Thread(target=self._httpd.serve_forever, daemon=True)
        self._thread.start()

    def stop(self):
        if self.simulator is not None:self.simulator.stop()
        if self._httpd is not None:
            self._httpd.shutdown()
            self._httpd.server_close()
            self._httpd = None
