"""Navigation 2.0 仿真调试面板的 Flask 应用工厂。"""

import json
import time
from typing import Any

from flask import Flask, jsonify, render_template, request

from .commands import CommandDispatcher
from .state_bridge import WebStateBridge


def create_app(runner: Any) -> Flask:
    """创建绑定指定仿真 Runner 的无全局状态 Flask 应用。"""
    bridge = WebStateBridge(runner)
    dispatcher = CommandDispatcher(runner)
    app = Flask(__name__, static_folder="static", template_folder="templates")

    @app.get("/")
    def index():
        """返回新的 Dashboard 页面。"""
        return render_template("dashboard.html")

    @app.get("/api/snapshot")
    def snapshot():
        """返回当前组合快照，供轮询和断线恢复使用。"""
        return jsonify(bridge.snapshot_payload())

    @app.post("/api/sim/<command>")
    def simulation_command(command: str):
        """处理一个白名单仿真命令。"""
        result = dispatcher.dispatch(command, request.get_json(silent=True) or {})
        return jsonify(result.to_dict()), (200 if result.ok else 400)

    try:
        from flask_sock import Sock
        sock = Sock(app)

        @sock.route("/ws")
        def websocket(connection):
            """只推送快照；WebSocket 不拥有控制命令权限。"""
            while True:
                connection.send(json.dumps(bridge.snapshot_payload(), ensure_ascii=False))
                time.sleep(0.1)
    except ImportError:
        pass
    return app

