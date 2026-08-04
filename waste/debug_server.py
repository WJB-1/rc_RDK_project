# -*- coding: utf-8 -*-

"""
RoboCup Rescue Brain - 综合可视化调试服务器 (V2.0)

功能:
- /              : 调试主页 (HTML + Canvas渲染)
- /ws            : WebSocket 实时数据流 (JSON + Base64图像)
- /snapshot      : HTTP 获取当前状态快照
- /api/cmd       : HTTP POST 发送控制指令 (用于调试面板)

主程序通过 update() 接口推送最新数据。
调试面板通过 /api/cmd 发送手动控制指令。
"""

import threading
import time
import json
import base64
from typing import Optional, Dict, List, Any, Callable

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = None
    np = None


# ============================================================
# HTML 前端页面
# ============================================================
HTML_TEMPLATE = """<!DOCTYPE html>
<html lang="zh-CN">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>RoboCup Rescue Brain - 可视化调试面板</title>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body {
            background: #0a0a0a;
            color: #eee;
            font-family: 'Segoe UI', 'Microsoft YaHei', sans-serif;
            height: 100vh;
            overflow: hidden;
        }
        .header {
            height: 40px;
            background: #1a1a2e;
            border-bottom: 2px solid #16213e;
            display: flex;
            align-items: center;
            padding: 0 20px;
            font-size: 16px;
            font-weight: bold;
        }
        .header .status-dot {
            width: 10px;
            height: 10px;
            border-radius: 50%;
            margin-right: 10px;
            background: #e74c3c;
            transition: background 0.3s;
        }
        .header .status-dot.connected { background: #2ecc71; }
        .header .fps { margin-left: auto; font-size: 12px; color: #888; }
        .main-container {
            display: flex;
            height: calc(100vh - 40px);
        }
        .left-panel {
            flex: 1;
            display: flex;
            flex-direction: column;
            padding: 10px;
            gap: 10px;
            min-width: 800px;
        }
        .right-panel {
            width: 380px;
            display: flex;
            flex-direction: column;
            padding: 10px;
            gap: 10px;
            border-left: 2px solid #16213e;
            flex-shrink: 0;
            overflow-y: auto;
        }
        .panel-title {
            font-size: 14px;
            color: #3498db;
            margin-bottom: 8px;
            font-weight: bold;
        }
        .video-container {
            flex: 0 0 auto;
            background: #111;
            border: 1px solid #333;
            border-radius: 4px;
            overflow: hidden;
            position: relative;
            height: 280px;
        }
        .video-container img {
            width: 100%;
            height: 100%;
            object-fit: contain;
        }
        .video-label {
            position: absolute;
            top: 5px;
            left: 5px;
            background: rgba(0,0,0,0.7);
            padding: 2px 8px;
            border-radius: 3px;
            font-size: 11px;
            color: #aaa;
        }
        .map-container {
            flex: 2;
            background: #111;
            border: 1px solid #333;
            border-radius: 4px;
            position: relative;
            overflow: hidden;
            min-height: 400px;
        }
        .map-container canvas {
            width: 100%;
            height: 100%;
        }

        /* 调试面板样式 */
        .debug-panel {
            background: #111;
            border: 1px solid #333;
            border-radius: 4px;
            padding: 12px;
            margin-bottom: 10px;
        }
        .control-grid {
            display: grid;
            grid-template-columns: repeat(3, 1fr);
            gap: 6px;
            margin-bottom: 10px;
        }
        .btn {
            padding: 8px 4px;
            border: none;
            border-radius: 4px;
            font-size: 12px;
            cursor: pointer;
            transition: opacity 0.2s;
            font-weight: bold;
        }
        .btn:hover { opacity: 0.8; }
        .btn:active { opacity: 0.6; }
        .btn-forward { background: #27ae60; color: #fff; }
        .btn-backward { background: #7f8c8d; color: #fff; }
        .btn-left { background: #f39c12; color: #fff; }
        .btn-right { background: #e67e22; color: #fff; }
        .btn-stop { background: #e74c3c; color: #fff; }
        .btn-action { background: #3498db; color: #fff; }
        .btn-mode {
            background: #2c3e50;
            color: #fff;
            border: 1px solid #3498db;
        }
        .btn-mode.active {
            background: #3498db;
        }
        .slider-group {
            margin: 8px 0;
        }
        .slider-group label {
            display: block;
            font-size: 11px;
            color: #888;
            margin-bottom: 4px;
        }
        .slider-row {
            display: flex;
            align-items: center;
            gap: 8px;
        }
        .slider-row input[type="range"] {
            flex: 1;
            height: 6px;
        }
        .slider-row .value-display {
            width: 50px;
            text-align: right;
            font-size: 12px;
            font-family: 'Consolas', monospace;
            color: #3498db;
        }
        .cmd-log {
            background: #0a0a0a;
            border: 1px solid #222;
            border-radius: 3px;
            padding: 6px;
            font-family: 'Consolas', monospace;
            font-size: 10px;
            line-height: 1.6;
            max-height: 120px;
            overflow-y: auto;
            margin-top: 8px;
        }
        .cmd-log-item {
            padding: 1px 0;
            border-bottom: 1px solid #1a1a1a;
        }
        .cmd-log-time { color: #666; margin-right: 6px; }
        .cmd-log-cmd { color: #3498db; }
        .section-divider {
            border-top: 1px solid #333;
            margin: 10px 0;
        }

        .telemetry-panel {
            background: #111;
            border: 1px solid #333;
            border-radius: 4px;
            padding: 10px;
            overflow-y: auto;
            max-height: 280px;
        }
        .telemetry-grid {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 6px 12px;
        }
        .metric {
            margin: 4px 0;
            font-size: 12px;
        }
        .metric .label {
            color: #888;
            font-size: 10px;
            margin-bottom: 1px;
        }
        .metric .value {
            font-weight: bold;
            font-size: 13px;
            font-family: 'Consolas', monospace;
        }
        .metric .value.ok { color: #2ecc71; }
        .metric .value.warn { color: #f39c12; }
        .metric .value.alert { color: #e74c3c; }
        .path-info {
            background: #111;
            border: 1px solid #333;
            border-radius: 4px;
            padding: 10px;
            flex: 1;
            overflow-y: auto;
            min-height: 100px;
            max-height: 160px;
        }
        .path-list {
            font-family: 'Consolas', monospace;
            font-size: 11px;
            line-height: 1.6;
        }
        .path-node {
            display: inline-block;
            padding: 1px 5px;
            margin: 1px;
            border-radius: 3px;
            background: #2c3e50;
            color: #ecf0f1;
            font-size: 10px;
        }
        .path-node.current { background: #e74c3c; color: #fff; }
        .path-node.visited { background: #27ae60; color: #fff; }
        .path-node.target { background: #f39c12; color: #fff; }
        .path-node.planned { background: #3498db; color: #fff; }
        .event-log {
            background: #111;
            border: 1px solid #333;
            border-radius: 4px;
            padding: 10px;
            flex: 2;
            overflow-y: auto;
            font-family: 'Consolas', monospace;
            font-size: 11px;
            line-height: 1.5;
            min-height: 120px;
            max-height: 180px;
        }
        .event-item {
            padding: 2px 0;
            border-bottom: 1px solid #1a1a1a;
        }
        .event-time { color: #666; margin-right: 8px; }
        .event-type { color: #3498db; margin-right: 8px; }
        .legend {
            display: flex;
            gap: 12px;
            padding: 4px 8px;
            font-size: 10px;
            background: rgba(26, 26, 46, 0.9);
            position: absolute;
            bottom: 0;
            left: 0;
            right: 0;
        }
        .legend-item { display: flex; align-items: center; gap: 5px; }
        .legend-dot {
            width: 10px;
            height: 10px;
            border-radius: 50%;
        }
    </style>
</head>
<body>
    <div class="header">
        <div class="status-dot" id="statusDot"></div>
        <span>RoboCup Rescue Brain - 可视化调试面板</span>
        <span class="fps" id="fpsDisplay">FPS: -- | 延迟: --ms</span>
    </div>
    <div class="main-container">
        <div class="left-panel">
            <div style="display: flex; gap: 10px; flex: 1;">
                <div class="video-container" style="flex: 1;">
                    <img id="segImage" src="" alt="车道分割">
                    <span class="video-label">BiSeNet 车道分割</span>
                </div>
            </div>
            <div class="map-container">
                <canvas id="mapCanvas"></canvas>
                <div class="legend">
                    <div class="legend-item"><div class="legend-dot" style="background:#e74c3c;"></div>当前位置</div>
                    <div class="legend-item"><div class="legend-dot" style="background:#27ae60;"></div>已打卡</div>
                    <div class="legend-item"><div class="legend-dot" style="background:#f39c12;"></div>下一目标</div>
                    <div class="legend-item"><div class="legend-dot" style="background:#3498db;"></div>规划路径</div>
                    <div class="legend-item"><div class="legend-dot" style="background:#95a5a6;"></div>未打卡</div>
                </div>
            </div>
        </div>
        <div class="right-panel">
            <!-- 调试控制面板 -->
            <div class="debug-panel">
                <div class="panel-title">手动控制</div>
                <div style="display: flex; gap: 6px; margin-bottom: 8px;">
                    <button class="btn btn-mode active" id="modeAuto" onclick="setMode('auto')">自动</button>
                    <button class="btn btn-mode" id="modeManual" onclick="setMode('manual')">手动</button>
                </div>
                <div class="control-grid">
                    <div></div>
                    <button class="btn btn-forward" id="btnForward" onmousedown="sendCmd('forward')" onmouseup="sendCmd('stop')" ontouchstart="sendCmd('forward')" ontouchend="sendCmd('stop')">前进</button>
                    <div></div>
                    <button class="btn btn-left" id="btnLeft" onmousedown="sendCmd('left')" onmouseup="sendCmd('stop')" ontouchstart="sendCmd('left')" ontouchend="sendCmd('stop')">左转</button>
                    <button class="btn btn-stop" id="btnStop" onclick="sendCmd('stop')">急停</button>
                    <button class="btn btn-right" id="btnRight" onmousedown="sendCmd('right')" onmouseup="sendCmd('stop')" ontouchstart="sendCmd('right')" ontouchend="sendCmd('stop')">右转</button>
                    <div></div>
                    <button class="btn btn-backward" id="btnBackward" onmousedown="sendCmd('backward')" onmouseup="sendCmd('stop')" ontouchstart="sendCmd('backward')" ontouchend="sendCmd('stop')">后退</button>
                    <div></div>
                </div>
                <div class="slider-group">
                    <label>线速度 vx (mm/s)</label>
                    <div class="slider-row">
                        <input type="range" id="vxSlider" min="0" max="500" value="300" oninput="updateSlider('vx', this.value)">
                        <span class="value-display" id="vxValue">300</span>
                    </div>
                </div>
                <div class="slider-group">
                    <label>角速度 wz (mrad/s)</label>
                    <div class="slider-row">
                        <input type="range" id="wzSlider" min="0" max="500" value="100" oninput="updateSlider('wz', this.value)">
                        <span class="value-display" id="wzValue">100</span>
                    </div>
                </div>
                <div class="section-divider"></div>
                <div style="display: flex; gap: 6px; flex-wrap: wrap;">
                    <button class="btn btn-action" onclick="sendCmd('reset_odom')">清零里程</button>
                    <button class="btn btn-action" onclick="sendCmd('turn_left_90')">左转90°</button>
                    <button class="btn btn-action" onclick="sendCmd('turn_right_90')">右转90°</button>
                </div>
                <div class="cmd-log" id="cmdLog">
                    <div style="color:#666;font-size:10px;">指令发送日志...</div>
                </div>
            </div>

            <div class="telemetry-panel">
                <div class="panel-title">实时状态</div>
                <div class="telemetry-grid">
                    <div class="metric">
                        <div class="label">Agent 状态</div>
                        <div class="value" id="agentState">--</div>
                    </div>
                    <div class="metric">
                        <div class="label">打卡进度</div>
                        <div class="value" id="progress">--</div>
                    </div>
                    <div class="metric">
                        <div class="label">当前位置 (mm)</div>
                        <div class="value" id="position">--</div>
                    </div>
                    <div class="metric">
                        <div class="label">航向角</div>
                        <div class="value" id="yaw">--</div>
                    </div>
                    <div class="metric">
                        <div class="label">当前节点</div>
                        <div class="value" id="currentNode">--</div>
                    </div>
                    <div class="metric">
                        <div class="label">目标节点</div>
                        <div class="value" id="targetNode">--</div>
                    </div>
                    <div class="metric">
                        <div class="label">横向偏移</div>
                        <div class="value" id="offsetMm">--</div>
                    </div>
                    <div class="metric">
                        <div class="label">路口检测</div>
                        <div class="value" id="intersection">--</div>
                    </div>
                </div>
            </div>
            <div class="path-info">
                <div class="panel-title">路径规划</div>
                <div class="path-list" id="pathList">等待数据...</div>
            </div>
            <div class="event-log" id="eventLog">
                <div class="panel-title">事件日志</div>
            </div>
        </div>
    </div>

    <script>
        // ============================================================
        // 全局状态
        // ============================================================
        let currentMode = 'auto';
        let vxSpeed = 300;
        let wzSpeed = 100;
        let cmdHistory = [];

        // ============================================================
        // 模式切换
        // ============================================================
        function setMode(mode) {
            currentMode = mode;
            document.getElementById('modeAuto').classList.toggle('active', mode === 'auto');
            document.getElementById('modeManual').classList.toggle('active', mode === 'manual');
            logCmd(`模式切换: ${mode === 'auto' ? '自动' : '手动'}`);
            sendApiCmd('set_mode', { mode: mode });
        }

        // ============================================================
        // 滑块更新
        // ============================================================
        function updateSlider(name, value) {
            if (name === 'vx') {
                vxSpeed = parseInt(value);
                document.getElementById('vxValue').textContent = vxSpeed;
            } else if (name === 'wz') {
                wzSpeed = parseInt(value);
                document.getElementById('wzValue').textContent = wzSpeed;
            }
        }

        // ============================================================
        // 发送指令
        // ============================================================
        function sendCmd(cmd) {
            if (currentMode === 'auto' && cmd !== 'stop') {
                // 自动模式下只允许急停
                if (!confirm('当前为自动模式，发送手动指令将切换到手动模式，确认？')) {
                    return;
                }
                setMode('manual');
            }

            const cmdMap = {
                'forward': { type: 'move', vx: vxSpeed, wz: 0 },
                'backward': { type: 'move', vx: -vxSpeed, wz: 0 },
                'left': { type: 'move', vx: 0, wz: wzSpeed },
                'right': { type: 'move', vx: 0, wz: -wzSpeed },
                'stop': { type: 'action', action: 'stop' },
                'reset_odom': { type: 'action', action: 'reset_odom' },
                'turn_left_90': { type: 'turn', angle: 90 },
                'turn_right_90': { type: 'turn', angle: -90 },
            };

            const payload = cmdMap[cmd];
            if (payload) {
                sendApiCmd(cmd, payload);
                logCmd(`发送: ${cmd}`);
            }
        }

        // ============================================================
        // API 调用
        // ============================================================
        async function sendApiCmd(cmd, payload) {
            try {
                const response = await fetch('/api/cmd', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ cmd, ...payload })
                });
                const result = await response.json();
                if (!result.ok) {
                    logCmd(`错误: ${result.error}`, 'error');
                }
            } catch (e) {
                logCmd(`网络错误: ${e.message}`, 'error');
            }
        }

        // ============================================================
        // 指令日志
        // ============================================================
        function logCmd(msg, type = 'info') {
            const time = new Date().toLocaleTimeString();
            const color = type === 'error' ? '#e74c3c' : '#3498db';
            cmdHistory.push({ time, msg, color });
            if (cmdHistory.length > 50) cmdHistory.shift();

            const container = document.getElementById('cmdLog');
            container.innerHTML = cmdHistory.map(item =>
                `<div class="cmd-log-item"><span class="cmd-log-time">${item.time}</span><span class="cmd-log-cmd" style="color:${item.color}">${item.msg}</span></div>`
            ).join('');
            container.scrollTop = container.scrollHeight;
        }

        // ============================================================
        // 地图渲染配置
        // ============================================================
        const MAP_CONFIG = {
            nodeRadius: 10,
            carRadius: 14,
        };

        function computeMapTransform(w, h, nodes) {
            if (!nodes || Object.keys(nodes).length === 0) {
                return { scale: 0.1, ox: w/2, oy: 30 };
            }
            const nodeList = Object.values(nodes);
            let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
            nodeList.forEach(n => {
                minX = Math.min(minX, n.x);
                maxX = Math.max(maxX, n.x);
                minY = Math.min(minY, n.y);
                maxY = Math.max(maxY, n.y);
            });
            const padding = 60;
            const availW = w - padding * 2;
            const availH = h - padding * 2;
            const rangeX = maxX - minX || 1;
            const rangeY = maxY - minY || 1;
            const scaleX = availW / rangeX;
            const scaleY = availH / rangeY;
            const scale = Math.min(scaleX, scaleY);
            const ox = w / 2 - (minX + maxX) / 2 * scale;
            const oy = padding + (availH - rangeY * scale) / 2 - minY * scale;
            return { scale, ox, oy };
        }

        // ============================================================
        // WebSocket 连接
        // ============================================================
        const ws = new WebSocket(`ws://${window.location.host}/ws`);
        let lastFrameTime = Date.now();
        let frameCount = 0;
        let fps = 0;

        ws.onopen = () => {
            console.log('WebSocket 已连接');
            document.getElementById('statusDot').classList.add('connected');
        };

        ws.onclose = () => {
            console.log('WebSocket 已断开');
            document.getElementById('statusDot').classList.remove('connected');
        };

        ws.onmessage = (event) => {
            const data = JSON.parse(event.data);
            const receiveTime = Date.now();
            const latency = receiveTime - data.timestamp;

            frameCount++;
            if (receiveTime - lastFrameTime >= 1000) {
                fps = frameCount;
                frameCount = 0;
                lastFrameTime = receiveTime;
            }
            document.getElementById('fpsDisplay').textContent =
                `FPS: ${fps} | 延迟: ${latency}ms`;

            if (data.seg_image) {
                document.getElementById('segImage').src = 'data:image/jpeg;base64,' + data.seg_image;
            }

            if (data.map_data) {
                nodes = data.map_data.nodes || {};
                edges = data.map_data.edges || [];
                renderMap(data);
            }

            updateTelemetry(data);

            if (data.planned_path) {
                updatePath(data);
            }

            if (data.events) {
                updateEventLog(data.events);
            }
        };

        // ============================================================
        // 地图渲染
        // ============================================================
        function renderMap(data) {
            const canvas = document.getElementById('mapCanvas');
            const ctx = canvas.getContext('2d');
            const dpr = window.devicePixelRatio || 1;
            const rect = canvas.getBoundingClientRect();

            canvas.width = rect.width * dpr;
            canvas.height = rect.height * dpr;
            ctx.scale(dpr, dpr);

            const w = rect.width;
            const h = rect.height;
            const transform = computeMapTransform(w, h, nodes);
            const s = transform.scale;
            const ox = transform.ox;
            const oy = transform.oy;

            ctx.fillStyle = '#111';
            ctx.fillRect(0, 0, w, h);

            ctx.strokeStyle = '#1a1a2e';
            ctx.lineWidth = 1;
            for (let x = 0; x < w; x += 50) {
                ctx.beginPath();
                ctx.moveTo(x, 0);
                ctx.lineTo(x, h);
                ctx.stroke();
            }
            for (let y = 0; y < h; y += 50) {
                ctx.beginPath();
                ctx.moveTo(0, y);
                ctx.lineTo(w, y);
                ctx.stroke();
            }

            function toCanvas(x_mm, y_mm) {
                return { x: ox + x_mm * s, y: oy + y_mm * s };
            }

            edges.forEach(edge => {
                const a = nodes[edge.node_a];
                const b = nodes[edge.node_b];
                if (!a || !b) return;

                const pa = toCanvas(a.x, a.y);
                const pb = toCanvas(b.x, b.y);

                ctx.strokeStyle = edge.is_tunnel ? '#e74c3c' : '#444';
                ctx.lineWidth = edge.is_tunnel ? 3 : 2;
                ctx.beginPath();
                ctx.moveTo(pa.x, pa.y);
                ctx.lineTo(pb.x, pb.y);
                ctx.stroke();

                if (edge.is_tunnel) {
                    const mx = (pa.x + pb.x) / 2;
                    const my = (pa.y + pb.y) / 2;
                    ctx.fillStyle = '#e74c3c';
                    ctx.font = '10px sans-serif';
                    ctx.fillText('T', mx - 3, my + 3);
                }
            });

            const visitedNodes = data.visited_nodes || [];
            const currentNode = data.current_node;
            const targetNode = data.target_node;
            const plannedPath = data.planned_path || [];

            Object.entries(nodes).forEach(([name, node]) => {
                const p = toCanvas(node.x, node.y);
                const isVisited = visitedNodes.includes(name);
                const isCurrent = name === currentNode;
                const isTarget = name === targetNode;
                const isPlanned = plannedPath.includes(name);

                let color = '#95a5a6';
                if (isCurrent) color = '#e74c3c';
                else if (isTarget) color = '#f39c12';
                else if (isVisited) color = '#27ae60';
                else if (isPlanned) color = '#3498db';

                const nr = Math.max(6, Math.min(14, s * 40));
                ctx.fillStyle = color;
                ctx.beginPath();
                ctx.arc(p.x, p.y, nr, 0, Math.PI * 2);
                ctx.fill();

                ctx.strokeStyle = '#fff';
                ctx.lineWidth = 1.5;
                ctx.stroke();

                const isJunction = name.startsWith('T') || name === 'J_START';
                const fontSize = isJunction ? Math.max(8, nr * 0.7) : Math.max(9, nr * 0.8);
                ctx.fillStyle = '#fff';
                ctx.font = `bold ${fontSize}px sans-serif`;
                ctx.textAlign = 'center';
                ctx.textBaseline = 'bottom';
                const shouldLabel = !isJunction || s > 0.15;
                if (shouldLabel) {
                    ctx.fillText(name, p.x, p.y - nr - 2);
                }
            });

            if (data.position) {
                const [x, y, yaw] = data.position;
                const p = toCanvas(x, y);

                const cr = Math.max(8, Math.min(18, s * 50));
                ctx.fillStyle = '#e74c3c';
                ctx.beginPath();
                ctx.arc(p.x, p.y, cr, 0, Math.PI * 2);
                ctx.fill();
                ctx.strokeStyle = '#fff';
                ctx.lineWidth = 2;
                ctx.stroke();

                const yawRad = (yaw * Math.PI) / 180;
                const lineLen = cr * 1.8;
                ctx.strokeStyle = '#f39c12';
                ctx.lineWidth = 2;
                ctx.beginPath();
                ctx.moveTo(p.x, p.y);
                ctx.lineTo(
                    p.x + Math.sin(yawRad) * lineLen,
                    p.y + Math.cos(yawRad) * lineLen
                );
                ctx.stroke();

                ctx.fillStyle = '#f39c12';
                ctx.font = `${Math.max(9, cr * 0.6)}px sans-serif`;
                ctx.fillText(`${yaw.toFixed(1)}°`, p.x + cr + 2, p.y + cr);
            }

            if (data.trajectory && data.trajectory.length > 0) {
                ctx.strokeStyle = 'rgba(46, 204, 113, 0.5)';
                ctx.lineWidth = 2;
                ctx.setLineDash([5, 5]);
                ctx.beginPath();
                data.trajectory.forEach((pos, i) => {
                    const p = toCanvas(pos[0], pos[1]);
                    if (i === 0) ctx.moveTo(p.x, p.y);
                    else ctx.lineTo(p.x, p.y);
                });
                ctx.stroke();
                ctx.setLineDash([]);
            }
        }

        // ============================================================
        // 遥测数据更新
        // ============================================================
        function updateTelemetry(data) {
            const stateEl = document.getElementById('agentState');
            stateEl.textContent = data.agent_state || '--';
            stateEl.className = 'value ' + (data.agent_state === 'FINISHED' ? 'ok' :
                data.agent_state === 'TURNING' ? 'warn' : '');

            if (data.position) {
                const [x, y, yaw] = data.position;
                document.getElementById('position').textContent = `${x.toFixed(0)}, ${y.toFixed(0)}`;
                document.getElementById('yaw').textContent = `${yaw.toFixed(1)}°`;
            }

            document.getElementById('currentNode').textContent = data.current_node || '--';
            document.getElementById('targetNode').textContent = data.target_node || '--';

            const offsetEl = document.getElementById('offsetMm');
            offsetEl.textContent = data.offset_mm !== undefined ? `${data.offset_mm.toFixed(1)} mm` : '--';
            offsetEl.className = 'value ' + (Math.abs(data.offset_mm || 0) > 100 ? 'alert' :
                Math.abs(data.offset_mm || 0) > 50 ? 'warn' : 'ok');

            const interEl = document.getElementById('intersection');
            interEl.textContent = data.is_intersection ? '检测到路口 ⚠' : '正常';
            interEl.className = 'value ' + (data.is_intersection ? 'warn' : 'ok');

            document.getElementById('progress').textContent =
                data.progress ? `${data.progress.visited}/${data.progress.total}` : '--';
        }

        // ============================================================
        // 路径更新
        // ============================================================
        function updatePath(data) {
            const container = document.getElementById('pathList');
            if (!data.planned_path || data.planned_path.length === 0) {
                container.textContent = '暂无规划路径';
                return;
            }

            const visited = data.visited_nodes || [];
            const current = data.current_node;
            const target = data.target_node;

            container.innerHTML = data.planned_path.map((node, i) => {
                let cls = 'path-node';
                if (node === current) cls += ' current';
                else if (node === target) cls += ' target';
                else if (visited.includes(node)) cls += ' visited';
                else cls += ' planned';

                const arrow = i < data.planned_path.length - 1 ? ' → ' : '';
                return `<span class="${cls}">${node}</span>${arrow}`;
            }).join('');
        }

        // ============================================================
        // 事件日志
        // ============================================================
        const MAX_EVENTS = 50;
        function updateEventLog(events) {
            const container = document.getElementById('eventLog');
            const title = container.querySelector('.panel-title');
            container.innerHTML = '';
            container.appendChild(title);

            events.slice(-MAX_EVENTS).forEach(evt => {
                const div = document.createElement('div');
                div.className = 'event-item';
                const time = new Date(evt.timestamp * 1000).toLocaleTimeString();
                div.innerHTML = `<span class="event-time">${time}</span>` +
                    `<span class="event-type">[${evt.type}]</span>${evt.message}`;
                container.appendChild(div);
            });

            container.scrollTop = container.scrollHeight;
        }

        window.addEventListener('resize', () => {});
    </script>
</body>
</html>
"""


class DebugWebServer:
    """
    综合可视化调试服务器 (V2.0)

    功能:
    1. BiSeNet 车道分割图 (WebSocket Base64)
    2. 地图 + 车辆实时坐标 (Canvas 渲染)
    3. 巡逻轨迹 + 规划路线 (WebSocket 实时推送)
    4. 手动控制面板 (HTTP API /api/cmd)
    """

    def __init__(self, host: str = "0.0.0.0", port: int = 5000,
                 cmd_callback: Optional[Callable[[str, Dict], None]] = None):
        """
        Args:
            host: 服务器地址
            port: 端口
            cmd_callback: 控制指令回调函数，接收 (cmd_name, payload_dict)
                         用于将前端手动指令传递给主程序
        """
        try:
            from flask import Flask, Response, render_template_string, jsonify, request
        except ImportError as e:
            raise ImportError(
                "缺少 Flask 依赖，请运行: pip install flask flask-sock"
            ) from e

        self.app = Flask(__name__)
        self.Response = Response
        self.render_template_string = render_template_string
        self.jsonify = jsonify
        self.request = request
        self.host = host
        self.port = port
        self._cmd_callback = cmd_callback

        # 数据锁
        self._lock = threading.Lock()

        # 最新数据缓存
        self._seg_frame: Optional[np.ndarray] = None
        self._offset_mm: float = 0.0
        self._is_intersection: bool = False

        # 导航状态
        self._agent_state: str = "IDLE"
        self._position: Optional[tuple] = None
        self._current_node: str = "START"
        self._target_node: Optional[str] = None
        self._planned_path: List[str] = []
        self._visited_nodes: List[str] = []
        self._trajectory: List[List[float]] = []
        self._events: List[Dict] = []
        self._progress: Optional[Dict] = None

        # 地图拓扑
        self._map_nodes: Dict = {}
        self._map_edges: List[Dict] = []

        # 手动指令日志
        self._cmd_log: List[Dict] = []

        self._register_routes()

    # ------------------------------------------------------------------
    # 供主程序调用的更新接口
    # ------------------------------------------------------------------
    def update(self, seg_frame: Optional[np.ndarray] = None,
               offset_mm: float = 0.0,
               is_intersection: bool = False):
        """更新感知层数据"""
        if np is not None and seg_frame is not None:
            with self._lock:
                self._seg_frame = seg_frame.copy()
        with self._lock:
            self._offset_mm = offset_mm
            self._is_intersection = is_intersection

    def update_navigation(self,
                          agent_state: str = None,
                          position: tuple = None,
                          current_node: str = None,
                          target_node: str = None,
                          planned_path: List[str] = None,
                          visited_nodes: List[str] = None,
                          progress: Dict = None,
                          event: Dict = None):
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
        """设置地图拓扑"""
        with self._lock:
            self._map_nodes = nodes
            self._map_edges = edges

    def set_cmd_callback(self, callback: Callable[[str, Dict], None]):
        """设置控制指令回调"""
        self._cmd_callback = callback

    # ------------------------------------------------------------------
    # 内部数据打包
    # ------------------------------------------------------------------
    def _pack_data(self) -> Dict:
        """打包数据为 JSON"""
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
                "cmd_log": self._cmd_log,
                "map_data": {
                    "nodes": self._map_nodes,
                    "edges": self._map_edges,
                } if self._map_nodes else None,
            }

            if self._seg_frame is not None and cv2 is not None:
                try:
                    # 缩放到 400x400 省带宽, 质量 40 足够看
                    h, w = self._seg_frame.shape[:2]
                    if w > 400 or h > 400:
                        scale = 400.0 / max(w, h)
                        thumb = cv2.resize(self._seg_frame, None, fx=scale, fy=scale,
                                          interpolation=cv2.INTER_NEAREST)
                    else:
                        thumb = self._seg_frame
                    _, buf = cv2.imencode('.jpg', thumb,
                                          [cv2.IMWRITE_JPEG_QUALITY, 40])
                    data["seg_image"] = base64.b64encode(buf).decode('ascii')
                except Exception:
                    data["seg_image"] = None
            else:
                data["seg_image"] = None

        return data

    # ------------------------------------------------------------------
    # 路由注册
    # ------------------------------------------------------------------
    def _register_routes(self):
        @self.app.route("/")
        def index():
            return self.render_template_string(HTML_TEMPLATE)

        @self.app.route("/snapshot")
        def snapshot():
            return self.jsonify(self._pack_data())

        @self.app.route("/api/cmd", methods=["POST"])
        def api_cmd():
            """接收前端控制指令"""
            try:
                data = self.request.get_json(force=True)
                if not data or 'cmd' not in data:
                    return self.jsonify({"ok": False, "error": "缺少 cmd 字段"})

                cmd = data['cmd']
                payload = {k: v for k, v in data.items() if k != 'cmd'}

                # 记录指令日志
                with self._lock:
                    self._cmd_log.append({
                        "timestamp": time.time(),
                        "cmd": cmd,
                        "payload": payload,
                    })
                    if len(self._cmd_log) > 100:
                        self._cmd_log = self._cmd_log[-100:]

                # 调用回调函数
                if self._cmd_callback:
                    self._cmd_callback(cmd, payload)

                return self.jsonify({"ok": True, "cmd": cmd, "payload": payload})

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
                        data = self._pack_data()
                        ws.send(json.dumps(data))
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

    def start_thread(self):
        t = threading.Thread(target=self.run, daemon=True)
        t.start()
