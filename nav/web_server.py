# -*- coding: utf-8 -*-
"""可视化 Web 服务器（标准库，无外部依赖）。

数据源（可同时提供多个）：
- ``recorder``：运行中的 Recorder → 实时模式（浏览器轮询 /api/live）。
- ``library``：RecordingLibrary → 回放库模式（自动扫描 results/，页面下拉选择）。
- ``recording``：单个录制 dict → 单文件回放（向后兼容）。

端点：
- ``GET /``                  → 三维四视图页面（viewer.html）
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
                            "..", "web", "viewer.html")


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
            })
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
                    idx = int(raw) if raw is not None else None
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


class VizServer:
    def __init__(self, recorder=None, recording=None, library=None, port: int = 8765):
        self.recorder = recorder
        self.recording = recording
        self.library = library
        self.port = int(port)
        self._httpd = None
        self._thread = None

    @property
    def url(self):
        return f"http://127.0.0.1:{self.port}/"

    def start(self):
        if self._httpd is not None:
            return
        _Handler.server_ref = self
        self._httpd = ThreadingHTTPServer(("127.0.0.1", self.port), _Handler)
        self._thread = threading.Thread(target=self._httpd.serve_forever, daemon=True)
        self._thread.start()

    def stop(self):
        if self._httpd is not None:
            self._httpd.shutdown()
            self._httpd.server_close()
            self._httpd = None
