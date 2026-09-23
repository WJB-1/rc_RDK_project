"""Navigation 2.0 仿真调试面板的 Flask 应用工厂。"""

import json
import time
from pathlib import Path
from typing import Any

from flask import Flask, Response, jsonify, render_template, request

from .commands import CommandDispatcher
from .state_bridge import WebStateBridge

_WEB_DIR = Path(__file__).parent
_NAVIGATION_2_TEMPLATE_DIR = _WEB_DIR / "versions" / "navigation-2.0-simulator" / "templates"
_NAVIGATION_2_STATIC_DIR = _WEB_DIR / "versions" / "navigation-2.0-simulator" / "static"


def create_app(runner: Any) -> Flask:
    """创建绑定指定仿真 Runner 的无全局状态 Flask 应用。"""
    bridge = WebStateBridge(runner)
    dispatcher = CommandDispatcher(runner)
    app = Flask(
        __name__,
        static_folder=str(_NAVIGATION_2_STATIC_DIR),
        template_folder=str(_NAVIGATION_2_TEMPLATE_DIR),
    )

    @app.get("/")
    def index():
        """返回新的 Dashboard 页面。"""
        hardware_motion = getattr(runner, "motion_port", None) is not None
        title = "Navigation 2.0 Hardware Debugger" if hardware_motion else "Navigation 2.0 Simulator"
        return render_template("simulator.html", dashboard_title=title, hardware_motion=hardware_motion)

    @app.get("/api/snapshot")
    def snapshot():
        """返回当前组合快照，供轮询和断线恢复使用。"""
        return jsonify(bridge.snapshot_payload())

    @app.get("/api/vision")
    def vision_preview():
        vision_runtime = getattr(runner, "vision_runtime", None)
        if vision_runtime is None:
            return jsonify({"ok": False, "error": "vision debug runtime is disabled"}), 404
        image = vision_runtime.jpeg(request.args.get("view", "overlay"))
        if image is None:
            return jsonify({"ok": False, "error": "vision frame is not available"}), 404
        return Response(image, mimetype="image/jpeg", headers={"Cache-Control": "no-store"})

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
