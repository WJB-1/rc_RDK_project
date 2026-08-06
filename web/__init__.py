"""
Web 可视化模块 (Web Dashboard Module)

统一管理所有 Web 推送、HTML 模板、页面生成。
所有需要 Web 渲染的脚本都通过此模块获取 HTML 和推送数据。

使用:
  from web import get_dashboard_html, WebPushServer

  # 创建 WebSocket 推送服务器
  web = WebPushServer(host="0.0.0.0", port=5000, cmd_callback=on_cmd)
  web.set_map_topology(nodes, edges)
  web.start()

  # 每帧推送数据
  web.update(seg_frame=debug_panel, offset_mm=12.5, is_intersection=False)

文件结构:
  web/
  ├── __init__.py            ← WebPushServer + get_dashboard_html
  └── static/                 ← (symlink) → debug_frontend/static/
      ├── css/dashboard.css
      └── js/dashboard.js
"""
import os
import json
import time
import base64
import threading
from pathlib import Path
from typing import Dict, List, Optional, Callable

import cv2
import numpy as np

# HTML 模板路径
_WEB_DIR = Path(__file__).parent
_TEMPLATES = _WEB_DIR / "templates"
_PROD_HTML = _TEMPLATES / "dashboard_prod.html"   # 自包含单文件 (生产)
_DEV_HTML = _TEMPLATES / "dashboard_dev.html"      # 引用外部 CSS/JS (开发)
_DEBUG_FRONTEND = _WEB_DIR.parent.parent / "debug_frontend"


def get_dashboard_html(mode="prod") -> str:
    """
    读取 dashboard HTML。

    Args:
        mode: "prod" → 自包含单文件, "dev" → 引用外部 CSS/JS
    """
    path = _PROD_HTML if mode == "prod" else _DEV_HTML
    if path.exists():
        return path.read_text(encoding='utf-8')
    return "<h1>dashboard HTML not found. Run: python debug_frontend/generate.py</h1>"


def regenerate_html():
    """调用 generate.py 重新生成 HTML"""
    import subprocess
    gen_script = _DEBUG_FRONTEND / "generate.py"
    if gen_script.exists():
        subprocess.run(["python", str(gen_script)], cwd=str(_PROJECT_ROOT.parent),
                       capture_output=True)
        return get_dashboard_html()
    return None


class WebPushServer:
    """
    WebSocket 实时数据推送服务器。

    职责:
    - 提供 Flask HTTP 服务（首页 HTML + /api/cmd 控制接口）
    - WebSocket 推送感知/导航数据到前端
    - 地图拓扑数据注入

    使用:
      web = WebPushServer(host="0.0.0.0", port=5000, cmd_callback=on_cmd)
      web.set_map_topology(nodes, edges)
      web.start()

      # 每帧
      web.update(seg_frame=debug_panel, offset_mm=12.5)
    """

    def __init__(self, host: str = "0.0.0.0", port: int = 5000,
                 cmd_callback: Optional[Callable[[str, Dict], None]] = None):
        try:
            from flask import Flask, render_template, render_template_string, jsonify, request
        except ImportError as e:
            raise ImportError("缺少 Flask: pip install flask flask-sock") from e

        self.app = Flask(__name__,
                         static_folder=str(_WEB_DIR / "static"),
                         template_folder=str(_TEMPLATES))
        self.render_template_string = render_template_string
        self.jsonify = jsonify
        self.request = request
        self.host = host
        self.port = port
        self._cmd_callback = cmd_callback

        # --- 数据缓存 ---
        self._lock = threading.Lock()
        self._seg_frame = None
        self._offset_mm = 0.0
        self._is_intersection = False
        self._quality_score = 0.0

        # 导航状态
        self._agent_state = "MANUAL"
        self._position = None
        self._current_node = "START"
        self._target_node = None
        self._planned_path = []
        self._visited_nodes = []
        self._trajectory = []
        self._events = []
        self._progress = None

        # 地图拓扑
        self._map_nodes = {}
        self._map_edges = []

        # 底图数据（一次性注入的 patrol_path 等）
        self._base_map_data = {}

        self._cmd_log = []
        self._auto_mode = False
        self._register_routes()

    # ------------------------------------------------------------------
    # 供主程序调用的更新接口
    # ------------------------------------------------------------------
    def update(self, seg_frame=None, offset_mm=0.0, is_intersection=False,
               quality_score=0.0):
        """更新感知层数据"""
        with self._lock:
            if seg_frame is not None:
                self._seg_frame = seg_frame.copy()
            self._offset_mm = offset_mm
            self._is_intersection = is_intersection
            self._quality_score = quality_score

    def update_navigation(self, agent_state=None, position=None, current_node=None,
                          target_node=None, planned_path=None, visited_nodes=None,
                          progress=None, event=None):
        """更新导航状态"""
        with self._lock:
            if agent_state is not None:
                self._agent_state = agent_state
            if position is not None:
                self._position = position
                self._trajectory.append([position[0], position[1]])
                if len(self._trajectory) > 1000:
                    self._trajectory = self._trajectory[-1000:]
            if current_node is not None:
                self._current_node = current_node
            if target_node is not None:
                self._target_node = target_node
            if planned_path is not None:
                self._planned_path = planned_path
            if visited_nodes is not None:
                self._visited_nodes = visited_nodes
            if progress is not None:
                self._progress = progress
            if event is not None:
                self._events.append(event)
                if len(self._events) > 200:
                    self._events = self._events[-200:]

    def set_map_topology(self, nodes: Dict, edges: List[Dict]):
        """注入地图拓扑数据"""
        with self._lock:
            self._map_nodes = nodes
            self._map_edges = edges

    def set_base_map_data(self, data: Dict):
        """注入底图静态数据（patrol_path, all_paths 等，由 generate.py 产生）"""
        with self._lock:
            self._base_map_data = dict(data)

    def set_cmd_callback(self, callback: Callable[[str, Dict], None]):
        self._cmd_callback = callback

    # ------------------------------------------------------------------
    # 内部
    # ------------------------------------------------------------------
    def _pack_data(self) -> Dict:
        with self._lock:
            data = {
                "timestamp": int(time.time() * 1000),
                "agent_state": self._agent_state,
                "position": self._position,
                "current_node": self._current_node,
                "target_node": self._target_node,
                "planned_path": self._planned_path,
                "visited_nodes": self._visited_nodes,
                "trajectory": self._trajectory,
                "progress": self._progress,
                "events": self._events,
                "offset_mm": self._offset_mm,
                "is_intersection": self._is_intersection,
                "quality_score": self._quality_score,
                "cmd_log": self._cmd_log,
                "map_data": {
                    "nodes": self._map_nodes,
                    "edges": self._map_edges,
                } if self._map_nodes else None,
            }
            if self._base_map_data:
                data["map_data"] = (data["map_data"] or {})
                data["map_data"].update({
                    "patrol_path": self._base_map_data.get("patrol_path", []),
                    "lane_width_mm": self._base_map_data.get("lane_width_mm", 200),
                    "field_size_mm": self._base_map_data.get("field_size_mm", [3200, 4400]),
                    "all_paths": self._base_map_data.get("all_paths", {}),
                    "all_dists": self._base_map_data.get("all_dists", {}),
                })

            if self._seg_frame is not None and cv2 is not None:
                try:
                    h, w = self._seg_frame.shape[:2]
                    if w > 400 or h > 400:
                        scale = 400.0 / max(w, h)
                        thumb = cv2.resize(self._seg_frame, None, fx=scale, fy=scale,
                                          interpolation=cv2.INTER_NEAREST)
                    else:
                        thumb = self._seg_frame
                    _, buf = cv2.imencode('.jpg', thumb, [cv2.IMWRITE_JPEG_QUALITY, 40])
                    data["seg_image"] = base64.b64encode(buf).decode('ascii')
                except Exception:
                    data["seg_image"] = None
            else:
                data["seg_image"] = None

        return data

    def _register_routes(self):
        # 地图数据 JSON (注入 HTML 模板的 map_data 变量)
        import json as _json
        map_data_json = _json.dumps({
            "nodes": self._map_nodes,
            "edges": self._map_edges,
            "lane_width_mm": 200,
            # patrol_path 等由 set_base_map_data 注入
        })

        @self.app.route("/simulator")
        def simulator():
            """独立模拟器页面"""
            path = _TEMPLATES / "simulator.html"
            if path.exists():
                return path.read_text(encoding='utf-8')
            return "<h1>simulator.html not found</h1>"

        @self.app.route("/")
        def index():
            """旧版兼容 — 同时注入静态地图数据"""
            with self._lock:
                md = {"nodes": self._map_nodes, "edges": self._map_edges, "lane_width_mm": 200}
                if self._base_map_data:
                    md.update({k: v for k, v in self._base_map_data.items()
                               if k not in ("all_paths", "all_dists")})
            return self.render_template_string(
                get_dashboard_html().replace(
                    '{% if map_data %}\n<script>\nwindow.__MAP_DATA__ = {{ map_data | safe }};\n</script>\n{% endif %}',
                    f'<script>\nwindow.__MAP_DATA__ = {_json.dumps(md)};\n</script>'
                )
            )

        @self.app.route("/v2")
        def index_v2():
            """新版模块化 HTML"""
            with self._lock:
                md = {
                    "nodes": self._map_nodes,
                    "edges": self._map_edges,
                    "lane_width_mm": 200,
                }
                if self._base_map_data:
                    md.update({
                        "patrol_path": self._base_map_data.get("patrol_path", []),
                        "all_paths": self._base_map_data.get("all_paths", {}),
                        "all_dists": self._base_map_data.get("all_dists", {}),
                        "field_size_mm": self._base_map_data.get("field_size_mm", [3200, 4400]),
                    })
            # render_template 已在 __init__ 中导入
            return render_template("dashboard.html", map_data=_json.dumps(md))

        @self.app.route("/snapshot")
        def snapshot():
            return self.jsonify(self._pack_data())

        @self.app.route("/api/cmd", methods=["POST"])
        def api_cmd():
            try:
                data = self.request.get_json(force=True)
                if not data or 'cmd' not in data:
                    return self.jsonify({"ok": False, "error": "缺少 cmd 字段"})
                cmd = data['cmd']
                payload = {k: v for k, v in data.items() if k != 'cmd'}
                with self._lock:
                    self._cmd_log.append({"timestamp": time.time(), "cmd": cmd, "payload": payload})
                    if len(self._cmd_log) > 100:
                        self._cmd_log = self._cmd_log[-100:]
                if self._cmd_callback:
                    self._cmd_callback(cmd, payload)
                return self.jsonify({"ok": True, "cmd": cmd, "payload": payload})
            except Exception as e:
                return self.jsonify({"ok": False, "error": str(e)})

        @self.app.route("/api/mode", methods=["POST"])
        def api_mode():
            """自动/手动模式切换"""
            try:
                data = self.request.get_json(force=True)
                mode = data.get("mode", "manual") if data else "manual"
                with self._lock:
                    if mode == "auto":
                        self._auto_mode = True
                    else:
                        self._auto_mode = False
                    self._cmd_log.append({"timestamp": time.time(),
                                          "cmd": "switch_mode",
                                          "payload": {"mode": mode}})
                if self._cmd_callback:
                    self._cmd_callback("auto_mode_start" if mode == "auto" else "auto_mode_stop",
                                       {"mode": mode})
                return self.jsonify({"ok": True, "mode": mode})
            except Exception as e:
                return self.jsonify({"ok": False, "error": str(e)})

        @self.app.route("/api/sim_plan", methods=["POST"])
        def api_sim_plan():
            """模拟器路径规划 API — 复用 Python 端 PathPlanner"""
            try:
                data = self.request.get_json(force=True)
                current_node = data.get("current_node", "START")
                visited = data.get("visited", [])
                blocked_edges = data.get("blocked_edges", [])

                try:
                    from ..navigation.map_topology import get_topology
                    from ..navigation.map_oracle import MapOracle
                    from ..navigation.path_planner import PathPlanner
                except ImportError:
                    from navigation.map_topology import get_topology
                    from navigation.map_oracle import MapOracle
                    from navigation.path_planner import PathPlanner

                topo = get_topology()
                # 同步模拟器的 visited 状态到拓扑
                for name in visited:
                    if name in topo.nodes:
                        topo.nodes[name].is_visited = True

                # 标记 blocked 边
                for edge_id in blocked_edges:
                    for edge in topo.edges:
                        if edge.edge_id == edge_id:
                            edge.is_blocked = True

                oracle = MapOracle(topo)
                planner = PathPlanner(oracle, topo)

                unvisited = [n for n in topo.nodes
                             if topo.nodes[n].node_type == "mission"
                             and not topo.nodes[n].is_visited]

                if not unvisited:
                    return self.jsonify({"edge_tasks": [], "total_distance_mm": 0,
                                         "node_sequence": [], "finished": True})

                result = planner.replan(current_node, unvisited, blocked_edges=set(blocked_edges))
                tasks = []
                for t in result.edge_tasks:
                    tasks.append({
                        "edge_id": t.edge_id,
                        "from_node": t.from_node,
                        "to_node": t.to_node,
                        "expected_yaw": t.expected_yaw,
                        "distance_mm": t.distance_mm,
                        "is_tunnel": t.is_tunnel,
                        "speed_limit_ms": t.speed_limit_ms,
                    })

                return self.jsonify({
                    "edge_tasks": tasks,
                    "total_distance_mm": result.total_distance_mm,
                    "node_sequence": result.node_sequence,
                    "finished": False,
                })
            except Exception as e:
                return self.jsonify({"ok": False, "error": str(e)})

        # WebSocket
        try:
            from flask_sock import Sock
            self.sock = Sock(self.app)

            @self.sock.route("/ws")
            def websocket(ws):
                while True:
                    try:
                        ws.send(json.dumps(self._pack_data()))
                        time.sleep(0.1)
                    except Exception:
                        break
        except ImportError:
            pass

    # ------------------------------------------------------------------
    # 启动
    # ------------------------------------------------------------------
    def run(self):
        self.app.run(host=self.host, port=self.port, threaded=True)

    def start(self):
        t = threading.Thread(target=self.run, daemon=True)
        t.start()
        print(f"Web 调试面板: http://{self.host if self.host != '0.0.0.0' else 'localhost'}:{self.port}")
