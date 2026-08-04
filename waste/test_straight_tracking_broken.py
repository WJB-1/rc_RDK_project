#!/usr/bin/env python3
"""
直道循迹测试脚本 — 摄像头 + 视觉偏移 → STM32 + Web 可视化

用法:
  python robocup_rescue_brain/test/test_straight_tracking.py
  python robocup_rescue_brain/test/test_straight_tracking.py --port COM3 --speed 250 --no-serial

功能:
  1. 打开摄像头, 运行 BiSeNet + IPM 流水线
  2. 打开串口连接 STM32
  3. 以固定速度直行, 每帧发送 CMD_LANE_OFFSET (0x06)
  4. 启动 Web 调试服务器 (端口 5001), 浏览器实时查看分割图+IPM鸟瞰图+偏移量
  5. 实时打印串口回传的里程计和状态数据

安全:
  - Ctrl+C → STOP + 关串口 + 释放摄像头
  - offset 限幅 ±200mm
  - --no-serial 模式不连下位机, 纯视觉调试
"""

import sys
import os
import time
import math
import signal
import argparse
import threading
import base64
import json
from pathlib import Path
from collections import deque

PROJECT_ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(PROJECT_ROOT))
if PROJECT_ROOT.name == "robocup_rescue_brain":
    sys.path.insert(0, str(PROJECT_ROOT.parent))

import cv2
import numpy as np
import yaml

from communication.stm32_protocol import (
    FRAME_HEAD_0, FRAME_HEAD_1,
    CmdID, ActionCode, FbID, StmStatusCode,
    encode_velocity, encode_action, encode_lane_offset,
    decode_frame, decode_odom, decode_status,
)
from hardware.camera import CameraManager
from perception.lane_tracker import LaneTracker
from navigation.map_topology import get_topology
from web import WebPushServer


# ================================================================
# 串口
    position: fixed; bottom: 16px; right: 16px; z-index: 999;
    background: rgba(26, 26, 46, 0.95); border: 1px solid #0f3460;
    border-radius: 8px; padding: 10px; width: 260px;
    font-size: 11px; color: #ccc;
    max-height: 320px; overflow-y: auto;
}
.pp-panel h3 { color: #e94560; margin-bottom: 6px; font-size: 12px; }
.pp-panel .row { display: flex; align-items: center; margin-bottom: 3px; }
.pp-panel .row label { width: 68px; flex-shrink: 0; color: #888; font-size: 10px; }
.pp-panel .row input[type=range] { flex: 1; height: 4px; }
.pp-panel .row .val { width: 36px; text-align: right; font-family: monospace; color: #00d4aa; font-size: 10px; }
.pp-panel button {
    width: 100%; margin-top: 4px; padding: 4px;
    background: #e94560; color: #fff; border: none; border-radius: 4px;
    cursor: pointer; font-size: 10px;
}
.pp-toggle {
    position: fixed; bottom: 16px; right: 16px; z-index: 1000;
    background: #0f3460; color: #fff; border: none;
    padding: 5px 12px; border-radius: 4px; cursor: pointer; font-size: 11px;
}

/* 调试日志框 — 地图区域下方横条 */
.log-viewer {
    background: rgba(10, 10, 10, 0.95); border-top: 2px solid #16213e;
    padding: 6px 12px; margin: 0 10px;
    font-family: 'Consolas', 'Courier New', monospace;
    font-size: 10px; line-height: 1.4;
    max-height: 120px; overflow-y: auto;
    color: #aaa;
}
.log-viewer .log-ipm { color: #00d4aa; }
.log-viewer .log-warn { color: #f39c12; }
.log-viewer .log-stat { color: #3498db; }
.log-viewer .log-err  { color: #e74c3c; }
</style>
<button class="pp-toggle" id="ppToggle" onclick="togglePP()">⚙ 预处理</button>
<div class="pp-panel" id="ppPanel" style="display:none;">
    <h3>图像预处理参数</h3>
    <div class="row"><label>对比度</label><input type="range" id="ppAlpha" min="0.5" max="3.0" step="0.1" value="1.5" oninput="ppUpdate()"><span class="val" id="ppAlphaV">1.5</span></div>
    <div class="row"><label>亮度</label><input type="range" id="ppBeta" min="-50" max="50" step="1" value="10" oninput="ppUpdate()"><span class="val" id="ppBetaV">10</span></div>
    <div class="row"><label>CLAHE</label><input type="range" id="ppClaheClip" min="0" max="5.0" step="0.5" value="2.0" oninput="ppUpdate()"><span class="val" id="ppClaheClipV">2.0</span></div>
    <div class="row"><label>CLAHE网格</label><input type="range" id="ppClaheGrid" min="4" max="16" step="4" value="8" oninput="ppUpdate()"><span class="val" id="ppClaheGridV">8</span></div>
    <div class="row"><label>锐化</label><input type="range" id="ppSharpen" min="0" max="1.0" step="0.05" value="0.0" oninput="ppUpdate()"><span class="val" id="ppSharpenV">0.0</span></div>
    <button onclick="ppSend()">应用参数</button>
    <div id="ppStatus" style="margin-top:4px;color:#00d4aa;"></div>
</div>

<!-- 调试日志框 -->
<div class="log-viewer" id="logViewer"></div>

<script>
function togglePP() {
    var p = document.getElementById('ppPanel');
    p.style.display = p.style.display === 'none' ? 'block' : 'none';
}
function ppUpdate() {
    document.getElementById('ppAlphaV').textContent = document.getElementById('ppAlpha').value;
    document.getElementById('ppBetaV').textContent = document.getElementById('ppBeta').value;
    document.getElementById('ppClaheClipV').textContent = document.getElementById('ppClaheClip').value;
    document.getElementById('ppClaheGridV').textContent = document.getElementById('ppClaheGrid').value;
    document.getElementById('ppSharpenV').textContent = document.getElementById('ppSharpen').value;
}
async function ppSend() {
    var params = {};
    ['alpha','beta','clahe_clip','clahe_grid','sharpen'].forEach(function(k){
        params[k] = parseFloat(document.getElementById(k==='sharpen'?'ppSharpen':k==='clahe_clip'?'ppClaheClip':k==='clahe_grid'?'ppClaheGrid':k==='beta'?'ppBeta':'ppAlpha').value);
    });
    try {
        var r = await fetch('/api/params', {
            method: 'POST',
            headers: {'Content-Type': 'application/json'},
            body: JSON.stringify({
                alpha: parseFloat(document.getElementById('ppAlpha').value),
                beta: parseInt(document.getElementById('ppBeta').value),
                clahe_clip: parseFloat(document.getElementById('ppClaheClip').value),
                clahe_grid: parseInt(document.getElementById('ppClaheGrid').value),
                sharpen: parseFloat(document.getElementById('ppSharpen').value),
            })
        });
        var d = await r.json();
        document.getElementById('ppStatus').textContent = '✓ 已应用';
        setTimeout(function(){ document.getElementById('ppStatus').textContent = ''; }, 1500);
    } catch(e) {
        document.getElementById('ppStatus').textContent = '✗ 发送失败';
    }
}
// 启动时从服务器拉取当前值
(async function(){
    try {
        var r = await fetch('/api/params');
        var p = await r.json();
        document.getElementById('ppAlpha').value = p.alpha;
        document.getElementById('ppBeta').value = p.beta;
        document.getElementById('ppClaheClip').value = p.clahe_clip;
        document.getElementById('ppClaheGrid').value = p.clahe_grid;
        document.getElementById('ppSharpen').value = p.sharpen;
        ppUpdate();
    } catch(e) {}
})();

// 调试日志 — 轮询 /api/logs 拉取
var LOG_MAX = 200;
var LOG_SEEN = 0;
function appendLog(msg, cls) {
    var el = document.getElementById('logViewer');
    if (!el) return;
    var line = document.createElement('div');
    line.className = cls || '';
    line.textContent = msg;
    el.appendChild(line);
    if (el.children.length > LOG_MAX) el.removeChild(el.firstChild);
    el.scrollTop = el.scrollHeight;
}
setInterval(function(){
    fetch('/api/logs').then(function(r){ return r.json(); }).then(function(data){
        if (!data.logs) return;
        for (var i = LOG_SEEN; i < data.logs.length; i++) {
            var l = data.logs[i];
            var cls = '';
            if (l.indexOf('[IPM') > -1) cls = 'log-ipm';
            else if (l.indexOf('WARNING') > -1 || l.indexOf('耗时') > -1) cls = 'log-warn';
            else if (l.indexOf('mm') > -1 && l.indexOf('+') > -1) cls = 'log-stat';
            appendLog(l, cls);
        }
        LOG_SEEN = data.logs.length;
    }).catch(function(){});
}, 500);
</script>
"""

# ================================================================
# 轻量 Web 调试服务器
# ================================================================
class LightDebugServer:
    """极简 HTTP + WebSocket 服务器, 单文件自包含"""

    def __init__(self, host="0.0.0.0", port=5001,
                 alpha=1.5, beta=10, clahe_clip=2.0, clahe_grid=8, sharpen=0.0):
        self.host = host
        self.port = port
        self._lock = threading.Lock()
        self._seg_frame = None
        self._offset_mm = 0.0
        self._is_intersection = False
        self._quality_score = 0.0
        self._odom_stats = {}

        # 预处理参数 (可运行时调节)
        self.preprocess = {
            "alpha": alpha,
            "beta": beta,
            "clahe_clip": clahe_clip,
            "clahe_grid": clahe_grid,
            "sharpen": sharpen,
        }

        # 调试日志缓冲区 (推送到 Web 端, 用 deque + 已发送游标)
        from collections import deque
        self._log_buffer = deque(maxlen=200)
        self._log_sent_idx = 0

        # 手动控制指令队列
        self._cmd_queue = deque(maxlen=20)

        # 地图拓扑
        topo = get_topology()
        self._map_nodes = {name: n.to_dict() for name, n in topo.nodes.items()}
        self._map_edges = [e.to_dict() for e in topo.edges]

    def log(self, line):
        """添加调试日志行"""
        import datetime
        ts = datetime.datetime.now().strftime("%H:%M:%S")
        with self._lock:
            self._log_buffer.append(f"{ts}  {line}")

    def get_params(self):
        with self._lock:
            return dict(self.preprocess)

    def drain_cmds(self):
        """取出并清空手动控制指令队列"""
        with self._lock:
            cmds = list(self._cmd_queue)
            self._cmd_queue.clear()
            return cmds

    def update(self, seg_frame=None, offset_mm=0.0, is_intersection=False,
               quality_score=0.0, odom_stats=None):
        with self._lock:
            if seg_frame is not None:
                self._seg_frame = seg_frame.copy()
            self._offset_mm = offset_mm
            self._is_intersection = is_intersection
            self._quality_score = quality_score
            if odom_stats:
                self._odom_stats = dict(odom_stats)

    def _pack(self):
        with self._lock:
            data = {
                "offset_mm": self._offset_mm,
                "is_intersection": self._is_intersection,
                "quality_score": self._quality_score,
                "odom_stats": self._odom_stats,
                "timestamp": int(time.time() * 1000),
                "logs": list(self._log_buffer)[self._log_sent_idx:],
            }
            self._log_sent_idx = len(self._log_buffer)  # 增量发送
            if self._seg_frame is not None:
                # 缩略图 400x400, 质量 40, 省带宽
                h, w = self._seg_frame.shape[:2]
                if w > 400 or h > 400:
                    scale = 400.0 / max(w, h)
                    thumb = cv2.resize(self._seg_frame, None, fx=scale, fy=scale,
                                      interpolation=cv2.INTER_NEAREST)
                else:
                    thumb = self._seg_frame
                _, buf = cv2.imencode('.jpg', thumb, [cv2.IMWRITE_JPEG_QUALITY, 40])
                data["seg_image"] = base64.b64encode(buf).decode('ascii')
            else:
                data["seg_image"] = None
        return data

    def _ws_handler(self, ws):
        import json as _json
        while True:
            try:
                ws.send(_json.dumps(self._pack()))
                time.sleep(0.1)
            except Exception:
                break

    def start_thread(self):
        try:
            from flask import Flask, render_template_string, Response
            from flask_sock import Sock
        except ImportError:
            print("需要安装: pip install flask flask-sock")
            return

        app = Flask(__name__)
        sock = Sock(app)
        server = self

        # 读取 HTML 模板文件
        html_path = PROJECT_ROOT / "index.html"
        if html_path.exists():
            html_content = html_path.read_text(encoding='utf-8')
        else:
            html_content = "<h1>index.html not found</h1>"

        # 注入预处理控制面板 CSS + HTML + JS
        html_content = html_content.replace('</body>', _PREPROCESS_PANEL + '\n</body>')

        from flask import make_response, jsonify
        @app.route("/")
        def index():
            return make_response(html_content)

        @app.route("/api/params", methods=["GET", "POST"])
        def api_params():
            try:
                if request.method == "POST":
                    data = request.get_json(force=True) or {}
                    with server._lock:
                        for k in ("alpha", "beta", "clahe_clip", "clahe_grid", "sharpen"):
                            if k in data:
                                server.preprocess[k] = float(data[k])
                    return jsonify({"ok": True, "params": server.get_params()})
                return jsonify(server.get_params())
            except Exception as e:
                import traceback
                traceback.print_exc()
                return jsonify({"ok": False, "error": str(e)})

        @app.route("/api/logs")
        def api_logs():
            with server._lock:
                logs = list(server._log_buffer)
            return jsonify({"logs": logs})

        @app.route("/api/cmd", methods=["POST"])
        def api_cmd():
            try:
                data = request.get_json(force=True) or {}
                cmd = data.get("cmd", "")
                payload = {k: v for k, v in data.items() if k != "cmd"}
                server._cmd_queue.append({"cmd": cmd, "payload": payload, "ts": time.time()})
                return jsonify({"ok": True, "cmd": cmd})
            except Exception as e:
                return jsonify({"ok": False, "error": str(e)})

        @sock.route("/ws")
        def websocket(ws):
            while True:
                try:
                    ws.send(json.dumps(server._pack()))
                    time.sleep(0.1)
                except Exception:
                    break

        t = threading.Thread(target=lambda: app.run(host=self.host, port=self.port, threaded=True), daemon=True)
        t.start()
        print(f"Web 调试面板: http://localhost:{self.port}")


# ================================================================
# 串口
# ================================================================
def open_serial(port="/dev/ttyUSB0", baudrate=115200):
    try:
        import serial
        ser = serial.Serial(port, baudrate, timeout=0.01, write_timeout=0.5)
        print(f"串口已连接: {port} @ {baudrate}bps")
        return ser
    except Exception as e:
        print(f"串口打开失败: {e}")
        return None


def read_feedback(ser):
    frames = []
    try:
        if ser and ser.is_open and ser.in_waiting > 0:
            data = ser.read(ser.in_waiting)
            buf = bytearray(data)
            while len(buf) >= 5:
                result = decode_frame(bytes(buf))
                if result is None:
                    idx = -1
                    for i in range(1, len(buf) - 1):
                        if buf[i] == FRAME_HEAD_0 and buf[i + 1] == FRAME_HEAD_1:
                            idx = i
                            break
                    if idx > 0:
                        buf = buf[idx:]
                    else:
                        break
                else:
                    cmd_id, payload, frame_total = result
                    frames.append((cmd_id, payload))
                    buf = buf[frame_total:]
    except Exception:
        pass
    return frames


# ================================================================
# 图像预处理
# ================================================================
def preprocess_frame(frame, alpha=1.5, beta=10, clahe_clip=2.0, clahe_grid=8, sharpen=0.0):
    """
    图像预处理: 对比度增强 + CLAHE + 可选锐化

    Args:
        frame: BGR 原始帧
        alpha: 对比度系数 (1.0=不变, >1 增强)
        beta: 亮度偏移
        clahe_clip: CLAHE 裁剪限制 (0=关闭)
        clahe_grid: CLAHE 网格
        sharpen: 锐化强度 (0=关闭)
    Returns:
        BGR 预处理后的帧
    """
    img = frame.copy()

    # 1. 对比度 + 亮度: dst = alpha * src + beta
    if alpha != 1.0 or beta != 0:
        img = cv2.convertScaleAbs(img, alpha=alpha, beta=beta)

    # 2. CLAHE (在 LAB 的 L 通道做, 比直接 BGR 更自然)
    if clahe_clip > 0:
        lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=clahe_clip, tileGridSize=(clahe_grid, clahe_grid))
        l = clahe.apply(l)
        lab = cv2.merge([l, a, b])
        img = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)

    # 3. 锐化
    if sharpen > 0:
        kernel = np.array([[-1, -1, -1],
                           [-1,  9, -1],
                           [-1, -1, -1]], dtype=np.float32) * sharpen + \
                 np.array([[0, 0, 0],
                           [0, 1, 0],
                           [0, 0, 0]], dtype=np.float32)
        img = cv2.filter2D(img, -1, kernel)
        img = np.clip(img, 0, 255).astype(np.uint8)

    return img


# ================================================================
# 主循环
# ================================================================
def main():
    parser = argparse.ArgumentParser(description="直道循迹测试 — 摄像头视觉 + STM32")
    parser.add_argument("--port", default="/dev/ttyUSB0", help="串口设备")
    parser.add_argument("--speed", type=int, default=300, help="前进速度 mm/s")
    parser.add_argument("--no-serial", action="store_true", help="不连接串口, 纯视觉调试")
    parser.add_argument("--web-port", type=int, default=5001, help="Web 调试端口")
    parser.add_argument("--config", default="config/settings.yaml", help="配置文件路径")
    # 图像预处理参数
    parser.add_argument("--alpha", type=float, default=1.5, help="对比度增强系数 (1.0=原图, 1.5=增强50%%)")
    parser.add_argument("--beta", type=int, default=10, help="亮度偏移 (-50~50, 正值提亮)")
    parser.add_argument("--clahe-clip", type=float, default=2.0, help="CLAHE 裁剪限制 (0=关闭, 推荐 1.5~3.0)")
    parser.add_argument("--clahe-grid", type=int, default=8, help="CLAHE 网格大小 (默认 8x8)")
    parser.add_argument("--sharpen", type=float, default=0, help="锐化强度 (0=关闭, 推荐 0.3~0.8)")
    parser.add_argument("--debug-bpu", action="store_true", help="打印 BPU 推理各阶段耗时")
    args = parser.parse_args()

    print("=" * 60)
    print("直道循迹测试 — 摄像头视觉 + STM32 偏移发送")
    print(f"  速度: {args.speed} mm/s")
    print(f"  串口: {'禁用' if args.no_serial else args.port}")
    print(f"  Web:  http://localhost:{args.web_port}")
    print(f"  预理: alpha={args.alpha} beta={args.beta} clahe={args.clahe_clip}/{args.clahe_grid} sharpen={args.sharpen}")
    print("=" * 60)

    # --- 加载配置 ---
    config_path = PROJECT_ROOT / args.config
    settings = {}
    if config_path.exists():
        with open(config_path, 'r', encoding='utf-8') as f:
            settings = yaml.safe_load(f) or {}
    else:
        settings = {"cameras": {"front": {"device_id": 0, "width": 1920, "height": 1080, "fps": 30}},
                    "segmentation": {}, "math_ipm": {}}

    # --- 初始化摄像头 ---
    print("\n[1/3] 初始化摄像头...")
    cam = CameraManager(settings)
    if not cam.initialize():
        print("摄像头初始化失败!")
        return

    # --- 初始化 LaneTracker ---
    print("[2/3] 初始化视觉追踪器...")
    tracker = LaneTracker(settings)
    # 开启 BPU 计时（不改源代码，直接 monkey-patch）
    if args.debug_bpu:
        _orig_infer = tracker.seg_engine.inference
        def _timed_infer(frame):
            return _orig_infer(frame, debug_timing=True)
        tracker.seg_engine.inference = _timed_infer
        print("  BPU 计时已开启")

    # --- Web 调试服务器 ---
    print("[3/3] 启动 Web 调试服务器...")
    web = WebPushServer(host="0.0.0.0", port=args.web_port)
    topo = get_topology()
    web.set_map_topology(
        nodes={name: n.to_dict() for name, n in topo.nodes.items()},
        edges=[e.to_dict() for e in topo.edges],
    )
    web.start()

    # 捕获 print 输出推到 Web 日志
    import builtins
    _orig_print = builtins.print
    def _tee_print(*args, **kwargs):
        msg = ' '.join(str(a) for a in args)
        web.log(msg)
        _orig_print(*args, **kwargs)
    builtins.print = _tee_print

    # --- 串口 ---
    ser = None
    if not args.no_serial:
        ser = open_serial(args.port)
        if ser is None:
            print("串口失败, 仅视觉模式")
            args.no_serial = True

    # --- 状态 ---
    running = True
    odom_stats = {"dist": 0, "yaw": 0.0, "speed": 0, "status": "UNKNOWN"}
    tx_count = 0
    rx_count = 0
    frame_count = 0

    def signal_handler(sig, frame):
        nonlocal running
        print("\n收到中断信号，停止...")
        running = False

    signal.signal(signal.SIGINT, signal_handler)

    # --- 发送前进指令 ---
    if not args.no_serial and ser:
        ser.write(encode_velocity(args.speed, 0))
        ser.flush()
        print(f"已发送: 直行 vx={args.speed} mm/s")

    # --- 主循环 ---
    print("\n开始循环 (Ctrl+C 停止)...")
    print("浏览器打开 Web 面板查看分割图和 IPM 鸟瞰图\n")
    print(f"{'帧':>6s}  {'offset':>8s}  {'质量':>6s}  {'路口':>5s}  {'距离':>8s}  {'航向':>8s}  {'TX/s':>6s}")
    print("-" * 70)

    t_start = time.time()
    last_print = t_start

    while running:
        loop_start = time.time()

        # 0. 处理 Web 端手动控制指令
        for c in web.drain_cmds():
            web.log(f"[CMD] {c['cmd']} {c['payload']}")
            if not args.no_serial and ser:
                if c['cmd'] in ('forward', 'backward'):
                    vx = c['payload'].get('vx', args.speed)
                    ser.write(encode_velocity(vx if c['cmd'] == 'forward' else -vx, 0))
                elif c['cmd'] in ('left', 'right'):
                    wz = c['payload'].get('wz', 100)
                    ser.write(encode_velocity(0, wz if c['cmd'] == 'left' else -wz))
                elif c['cmd'] == 'stop':
                    ser.write(encode_action(ActionCode.STOP))
                ser.flush()

        # 1. 读下位机反馈
        if not args.no_serial and ser:
            for cmd_id, payload in read_feedback(ser):
                rx_count += 1
                if cmd_id == FbID.ODOMETRY:
                    parsed = decode_odom(payload)
                    if parsed:
                        odom_stats["dist"] = parsed[0]
                        odom_stats["yaw"] = parsed[1] or 0.0
                        odom_stats["speed"] = parsed[2] or 0
                elif cmd_id == FbID.STATUS:
                    parsed = decode_status(payload)
                    if parsed:
                        odom_stats["status"] = str(parsed)

        # 2. 摄像头帧 → 视觉处理
        frames = cam.get_frames()
        front = frames.get("front")

        offset_mm = 0.0
        is_intersection = False
        quality_score = 1.0
        debug_frame = None

        if front is not None:
            # 预处理: 从 Web 端读取实时参数
            pp = web.get_params()
            front_proc = preprocess_frame(
                front,
                alpha=pp["alpha"],
                beta=pp["beta"],
                clahe_clip=pp["clahe_clip"],
                clahe_grid=pp["clahe_grid"],
                sharpen=pp["sharpen"],
            )
            offset_mm, is_intersection, debug_frame = tracker.process(front_proc)
            lane_state = tracker.last_lane_state
            if lane_state:
                quality_score = lane_state.get("quality_score", 1.0)

            # 3. 发送偏移量到 STM32
            if not args.no_serial and ser:
                ser.write(encode_lane_offset(offset_mm))
                ser.flush()
            tx_count += 1

        # 4. 推送 Web 面板
        web.update(seg_frame=debug_frame, offset_mm=offset_mm,
                   is_intersection=is_intersection, quality_score=quality_score,
                   odom_stats=odom_stats)

        frame_count += 1

        # 5. 每秒打印
        current_time = time.time()
        if current_time - last_print >= 1.0:
            elapsed = current_time - t_start
            fps = frame_count / (current_time - last_print)
            print(
                f"{frame_count:5d}  {offset_mm:+7.1f}mm  "
                f"{quality_score:.2f}  {'⚠' if is_intersection else ' '}   "
                f"{odom_stats['dist']:7d}mm  {odom_stats['yaw']:+7.1f}°  "
                f"{fps:5.1f}"
            )
            frame_count = 0
            last_print = current_time

        # 6. 控频 (~20Hz 视觉)
        elapsed_loop = time.time() - loop_start
        sleep_time = max(0, 0.05 - elapsed_loop)
        if sleep_time > 0:
            time.sleep(sleep_time)

    # --- 清理 ---
    print("\n正在停止...")

    if not args.no_serial and ser:
        ser.write(encode_action(ActionCode.STOP))
        ser.flush()
        print("已发送: STOP")

    cam.release()
    if not args.no_serial and ser:
        ser.close()

    elapsed = time.time() - t_start
    print(f"\n--- 测试结束 ---")
    print(f"  运行: {elapsed:.1f}s | 发送: {tx_count} 帧 ({(tx_count/max(elapsed,0.01)):.1f} Hz) | 接收: {rx_count} 帧")


if __name__ == "__main__":
    main()
