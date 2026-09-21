"""

# Navigation 2.0 快照桥接和受控应用工厂；旧版 WebPushServer 保持兼容。
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

# Navigation 2.0 快照桥接和受控应用工厂；旧版 WebPushServer 保持兼容。
from .app import create_app
from .commands import CommandDispatcher, CommandResult, SimulationCommand
from .state_bridge import WebStateBridge

# HTML 模板路径
_WEB_DIR = Path(__file__).parent
_VERSIONS_DIR = _WEB_DIR / "versions"
_NAVIGATION_2_TEMPLATES = _VERSIONS_DIR / "navigation-2.0" / "templates"
_LEGACY_DASHBOARD_TEMPLATES = _VERSIONS_DIR / "legacy-dashboard-1.0" / "templates"
_LEGACY_SIMULATOR_TEMPLATES = _VERSIONS_DIR / "legacy-simulator-1.0" / "templates"
_LEGACY_SIMULATOR_STATIC = _VERSIONS_DIR / "legacy-simulator-1.0" / "static"
_TEMPLATES = _WEB_DIR / "templates"
_PROD_HTML = _TEMPLATES / "dashboard_prod.html"   # 自包含单文件 (生产)
_DEV_HTML = _TEMPLATES / "dashboard_dev.html"      # 引用外部 CSS/JS (开发)
_PROD_HTML = _LEGACY_DASHBOARD_TEMPLATES / "dashboard_prod.html"
_DEV_HTML = _LEGACY_DASHBOARD_TEMPLATES / "dashboard_dev.html"
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
                 cmd_callback: Optional[Callable[[str, Dict], None]] = None,
                 mode: str = "sim"):
        """
        Args:
            host: 监听地址
            port: 监听端口
            cmd_callback: 真机控制指令回调（仅实机模式需要）
            mode: 运行模式
                - "sim"  (默认): 仿真/调试模式，注册 /simulator 页与 /api/sim/* 仿真接口
                - "real": 实机模式，不注册任何仿真接口，只保留真机控制 (/api/cmd、/api/mode)
        """
        try:
            from flask import Flask, render_template, render_template_string, jsonify, request
        except ImportError as e:
            raise ImportError("缺少 Flask: pip install flask flask-sock") from e

        self.app = Flask(__name__,
                         static_folder=str(_LEGACY_SIMULATOR_STATIC),
                         static_url_path="/legacy-static",
                         template_folder=str(_NAVIGATION_2_TEMPLATES))
        self.render_template_string = render_template_string
        self.jsonify = jsonify
        self.request = request
        self.host = host
        self.port = port
        self.mode = mode            # "sim" | "real"
        self._cmd_callback = cmd_callback

        # --- 数据缓存 ---
        self._lock = threading.Lock()
        self._seg_frame = None
        self._offset_mm = 0.0
        self._is_intersection = False
        self._quality_score = 0.0
        self._semantic_gate_available = False
        self._semantic_gate_enabled = False

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

        # 仿真状态
        self._sim_running = False
        self._sim_seed = 0
        self._sim_speed = 1.0
        self._sim_engine = None
        self._sim_scene = None

        self._register_routes()

    # ------------------------------------------------------------------
    # 供主程序调用的更新接口
    # ------------------------------------------------------------------
    def update(self, seg_frame=None, offset_mm=0.0, is_intersection=False,
               quality_score=0.0, semantic_gate_available=None,
               semantic_gate_enabled=None):
        """更新感知层数据"""
        with self._lock:
            if seg_frame is not None:
                self._seg_frame = seg_frame.copy()
            self._offset_mm = offset_mm
            self._is_intersection = is_intersection
            self._quality_score = quality_score
            if semantic_gate_available is not None:
                self._semantic_gate_available = bool(semantic_gate_available)
            if semantic_gate_enabled is not None:
                self._semantic_gate_enabled = bool(semantic_gate_enabled)

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

    def set_sim_state(self, running=False, seed=0, speed=1.0, scene=None):
        """更新仿真状态（供 SimEngine 回调）"""
        with self._lock:
            self._sim_running = running
            self._sim_seed = seed
            self._sim_speed = speed
            self._sim_scene = scene

    def set_sim_engine(self, engine):
        """注入仿真引擎引用（用于 API 控制）"""
        self._sim_engine = engine

    # ------------------------------------------------------------------
    # 内部
    # ------------------------------------------------------------------
    def _on_sim_update(self, snapshot: dict):
        """SimEngine 状态更新回调 — 将仿真数据同步到 WebPushServer"""
        with self._lock:
            self._agent_state = snapshot.get("agent_state", "")
            pos = snapshot.get("pose", {})
            self._position = (
                pos.get("x_mm", 0),
                pos.get("y_mm", 0),
                pos.get("yaw_deg", 0),
            )
            self._current_node = snapshot.get("current_node", "")
            self._target_node = snapshot.get("target_node", "")
            self._planned_path = snapshot.get("planned_path", [])
            self._visited_nodes = snapshot.get("visited_nodes", [])
            # 回写发现/侦查状态到 _sim_scene，使 _pack_data() 里 scene.to_dict()
            # 携带实时 discovered/recon/obstacle（否则前端统计与标签永远停留在初始空态）
            if self._sim_scene is not None:
                disc = snapshot.get("discovered_culverts")
                recon = snapshot.get("recon_culverts")
                obs = snapshot.get("discovered_obstacles")
                if disc is not None:
                    self._sim_scene.discovered_culverts = set(disc)
                if recon is not None:
                    self._sim_scene.recon_culverts = set(recon)
                if obs is not None:
                    self._sim_scene.discovered_obstacles = set(obs)
            mp = snapshot.get("mission_progress", {})
            self._progress = {
                "visited": mp.get("visited", 0),
                "total": mp.get("total", 12),
            }
            # 同步节点的 is_visited（打卡状态），否则详情页「打卡」字段永远停在初始 False
            if self._sim_engine:
                for name, node_dict in self._map_nodes.items():
                    try:
                        topo_node = self._sim_engine.topo.nodes.get(name)
                        if topo_node:
                            node_dict["is_visited"] = topo_node.is_visited
                    except Exception:
                        pass
            # 更新边状态（blocked/culvert → map_data.edges 的 has_culvert/is_blocked 字段）
            # 从 topo 同步
            if self._sim_engine:
                for i, edge_dict in enumerate(self._map_edges):
                    try:
                        topo_edge = None
                        for e in self._sim_engine.topo.edges:
                            if e.edge_id == edge_dict.get("edge_id"):
                                topo_edge = e
                                break
                        if topo_edge:
                            self._map_edges[i]["is_blocked"] = topo_edge.is_blocked
                            self._map_edges[i]["has_culvert"] = topo_edge.has_culvert
                            self._map_edges[i]["is_reconned"] = topo_edge.is_reconned
                            self._map_edges[i]["visit_count"] = topo_edge.visit_count
                    except Exception:
                        pass

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
                "semantic_gate_available": self._semantic_gate_available,
                "semantic_gate_enabled": self._semantic_gate_enabled,
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

            # 仿真状态
            if self._sim_scene is not None:
                data["sim"] = {
                    "running": self._sim_running,
                    "seed": self._sim_seed,
                    "speed": self._sim_speed,
                    "scene": self._sim_scene.to_dict() if self._sim_scene else None,
                }
            else:
                data["sim"] = None

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

        if self.mode == "sim":
            @self.app.route("/simulator")
            def simulator():
                """独立模拟器页面（仅仿真/调试模式）"""
                path = _LEGACY_SIMULATOR_TEMPLATES / "simulator.html"
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
            html = get_dashboard_html()
            if self.mode == "real":
                # 实机首页剔除「内置模拟器」面板：从 <!-- 模拟器控制 --> 到
                # <!-- 手动控制面板 --> 之间的整块移除（含起始锚点行，不含结束锚点行）。
                lines = html.splitlines()
                out, skipping = [], False
                for ln in lines:
                    if "模拟器控制" in ln:
                        skipping = True
                        continue
                    if skipping and "手动控制面板" in ln:
                        skipping = False
                    if not skipping:
                        out.append(ln)
                html = "\n".join(out)
            return self.render_template_string(
                html.replace(
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
                    from navigation.domain.topology import get_topology
                    from navigation.planning.map_oracle import MapOracle
                    from navigation.planning.path_planner import PathPlanner
                except ImportError:
                    from ..navigation.map_topology import get_topology
                    from ..navigation.map_oracle import MapOracle
                    from ..navigation.path_planner import PathPlanner

                topo = get_topology()
                # 每次请求重置访问状态，避免上次请求污染全局单例
                topo.reset_visit_status()
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
                             if topo.nodes[n].has_rfid
                             and not topo.nodes[n].is_visited]

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
                    "finished": (not tasks),  # 已在 START 且全部完成
                    "returning_to_start": result.returning_to_start,
                })
            except Exception as e:
                return self.jsonify({"ok": False, "error": str(e)})

        @self.app.route("/api/sim/start", methods=["POST"])
        def api_sim_start():
            """启动仿真"""
            try:
                data = self.request.get_json(force=True) or {}
                seed = data.get("seed", 42)
                speed_multiplier = data.get("speed_multiplier", 1.0)
                visible_range_mm = data.get("visible_range_mm", 400)

                try:
                    from navigation.simulation.scene.scene_generator import generate_scene
                    from navigation.domain.topology import get_topology
                    from navigation.sim_engine import SimEngine
                except ImportError:
                    import sys as _sys, os as _os
                    _parent = str(_WEB_DIR.parent)
                    if _parent not in _sys.path:
                        _sys.path.insert(0, _parent)
                    from navigation.simulation.scene.scene_generator import generate_scene
                    from navigation.domain.topology import get_topology
                    from navigation.sim_engine import SimEngine

                topo = get_topology()
                scene = generate_scene(seed=seed)

                engine = SimEngine(scene, topo, on_state_update=self._on_sim_update)
                engine._vision._visible_range_mm = visible_range_mm
                self.set_sim_engine(engine)
                self.set_sim_state(running=True, seed=seed, speed=speed_multiplier, scene=scene)

                engine.run_async(speed_multiplier=speed_multiplier)

                return self.jsonify({
                    "ok": True,
                    "seed": seed,
                    "scene": scene.to_dict(),
                })
            except Exception as e:
                import traceback as _tb
                return self.jsonify({"ok": False, "error": str(e), "traceback": str(_tb.format_exc())})

        @self.app.route("/api/sim/scene/generate", methods=["POST"])
        def api_sim_scene_generate():
            """仅生成场景（不启动仿真），返回地面真值供前端可视化"""
            try:
                data = self.request.get_json(force=True) or {}
                seed = data.get("seed", None)
                if seed is None:
                    import random as _rnd
                    seed = _rnd.randint(0, 100000)

                try:
                    from navigation.simulation.scene.scene_generator import generate_scene
                    from navigation.domain.topology import get_topology
                except ImportError:
                    import sys as _sys
                    _parent = str(_WEB_DIR.parent)
                    if _parent not in _sys.path:
                        _sys.path.insert(0, _parent)
                    from navigation.simulation.scene.scene_generator import generate_scene
                    from navigation.domain.topology import get_topology

                topo = get_topology()
                scene = generate_scene(seed=seed)
                self.set_sim_state(running=False, seed=seed, speed=1.0, scene=scene)

                # 构建前端需要的边信息（哪些是障碍边、哪些是涵洞边）
                edges_info = []
                for edge in topo.edges:
                    edges_info.append({
                        "edge_id": edge.edge_id,
                        "node_a": edge.node_a,
                        "node_b": edge.node_b,
                        "is_tunnel": edge.is_tunnel,
                        "is_obstacle": edge.edge_id in scene.obstacle_edge_ids,
                        "is_culvert": edge.edge_id in scene.culvert_edge_ids,
                        "obstacle_offset": scene.obstacle_offsets.get(edge.edge_id),
                        "culvert_offset": scene.culvert_offsets.get(edge.edge_id),
                    })

                return self.jsonify({
                    "ok": True,
                    "seed": seed,
                    "scene": scene.to_dict(),
                    "scene_edges": edges_info,
                })
            except Exception as e:
                import traceback as _tb
                return self.jsonify({"ok": False, "error": str(e), "traceback": str(_tb.format_exc())})

        @self.app.route("/api/sim/stop", methods=["POST"])
        def api_sim_stop():
            """停止仿真"""
            try:
                if self._sim_engine:
                    self._sim_engine.stop()
                self.set_sim_state(running=False)
                return self.jsonify({"ok": True})
            except Exception as e:
                return self.jsonify({"ok": False, "error": str(e)})

        @self.app.route("/api/sim/pause", methods=["POST"])
        def api_sim_pause():
            """暂停仿真"""
            try:
                if self._sim_engine:
                    self._sim_engine.pause()
                with self._lock:
                    self._sim_running = True
                return self.jsonify({"ok": True})
            except Exception as e:
                return self.jsonify({"ok": False, "error": str(e)})

        @self.app.route("/api/sim/resume", methods=["POST"])
        def api_sim_resume():
            """恢复仿真"""
            try:
                if self._sim_engine:
                    self._sim_engine.resume()
                return self.jsonify({"ok": True})
            except Exception as e:
                return self.jsonify({"ok": False, "error": str(e)})

        @self.app.route("/api/sim/step", methods=["POST"])
        def api_sim_step():
            """单步执行仿真"""
            try:
                if self._sim_engine is None:
                    return self.jsonify({"ok": False, "error": "仿真未启动"})
                if not self._sim_engine.is_running():
                    self._sim_engine.start()
                result = self._sim_engine.step()
                snapshot = self._sim_engine.get_state_snapshot()
                summary = self._sim_engine.get_summary()
                return self.jsonify({
                    "ok": True,
                    "has_next": result,
                    "state": summary["state"],
                    "snapshot": snapshot,
                    "summary": summary,
                })
            except Exception as e:
                return self.jsonify({"ok": False, "error": str(e)})

        @self.app.route("/api/sim/scene", methods=["GET"])
        def api_sim_scene():
            """获取当前场景真值"""
            try:
                if self._sim_scene is None:
                    return self.jsonify({"ok": False, "error": "无仿真场景"})
                summary = self._sim_engine.get_summary() if self._sim_engine else None
                return self.jsonify({
                    "ok": True,
                    "scene": self._sim_scene.to_dict(),
                    "summary": summary,
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

        # 实机模式：阻断一切仿真接口（路由隔离，与 run_dashboard.py 的模拟面板彻底区分）
        if self.mode == "real":
            @self.app.before_request
            def _block_sim_endpoints():
                from flask import request as _req, abort as _abort
                p = _req.path
                if p == "/simulator" or p.startswith("/api/sim"):
                    _abort(404)

    # ------------------------------------------------------------------
    # 启动
    # ------------------------------------------------------------------
    def run(self):
        self.app.run(host=self.host, port=self.port, threaded=True)

    def start(self):
        t = threading.Thread(target=self.run, daemon=True)
        t.start()
        print(f"Web 调试面板: http://{self.host if self.host != '0.0.0.0' else 'localhost'}:{self.port}")
