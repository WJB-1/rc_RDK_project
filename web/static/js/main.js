
/**
 * RoboCup Rescue Brain — 可视化调试面板 核心脚本
 *
 * 两种运行模式:
 *   1. 线上模式: 连接 debug_server.py 提供的 WebSocket (/ws) 实时推送数据
 *   2. 独立模式: 直接在浏览器打开 HTML 文件，内置模拟器驱动渲染
 *
 * 地图拓扑数据由 HTML 页面中的 <script> 标签注入 window.__MAP_DATA__。
 */

// 全局状态
let currentMode = 'auto';
let vxSpeed = 300;
let wzSpeed = 100;
let cmdHistory = [];

// canvas 引用（拖拽用）
let canvas = null;

// 地图数据（由 HTML 注入）
let gNodes = {};
let gEdges = [];

// 小车拖拽状态
let carDragging = false;
let carDragMode = null;
let carDragStart = null;
let carPosBeforeDrag = null;
let lastMapTransform = null;  // 缓存当前帧的变换参数，供拖拽用

// 模拟状态
let simRunning = false;
let simInterval = null;
let sim = {
    pos: { x: 0, y: 0, yaw: 0 },
    odom: 0,
    state: 'IDLE',
    currentNode: 'START',
    targetNode: null,
    visited: [],
    path: [],
    pathIndex: 0,
    trajectory: [],
    events: [],
    approachingUntil: 0,
};

// 初始化
function initDashboard() {
    // 加载地图拓扑数据
    if (window.__MAP_DATA__) {
        gNodes = window.__MAP_DATA__.nodes || {};
        gEdges = window.__MAP_DATA__.edges || [];
        sim.path = window.__MAP_DATA__.patrol_path || [];
    }
    updateStats();

    // 启用小车拖拽
    setupCarDrag();

    // 填充起始位置下拉框
    const startSelect = document.getElementById('simStartNode');
    if (startSelect && Object.keys(gNodes).length > 0) {
        const nodeNames = Object.keys(gNodes).sort((a, b) => {
            // mission 排前面, junction 排后面
            const ta = gNodes[a].type, tb = gNodes[b].type;
            if (ta === 'base') return -1;
            if (tb === 'base') return 1;
            if (ta === 'mission' && tb !== 'mission') return -1;
            if (ta !== 'mission' && tb === 'mission') return 1;
            return a.localeCompare(b);
        });
        startSelect.innerHTML = nodeNames.map(n => {
            const t = gNodes[n].type;
            const label = t === 'start' ? '起点' : t === 'mission' ? '任务点' : '路口';
            return `<option value="${n}">${n} (${label})</option>`;
        }).join('');
    }

    // 先画一次地图（延迟确保 Canvas 已渲染）
    setTimeout(() => {
        if (Object.keys(gNodes).length > 0) {
            drawMap(gNodes, gEdges, null, [], null, null, [], null);
        }
    }, 100);

    // 检测是否在线（WebSocket 可用）
    const isFileProtocol = window.location.protocol === 'file:';
    if (isFileProtocol) {
        // 独立模式 — 启用模拟器
        document.getElementById('statusDot').classList.add('connected');
        document.getElementById('fpsDisplay').textContent = '独立模式 | 模拟器就绪';
        showSimControls(true);
        setTimeout(() => {
            updateTelemetryFromSim();
            renderMapFromSim();
        }, 150);
    } else {
        // 线上模式 — 连接 WebSocket, 同时先画静态地图
        connectWebSocket();
        showSimControls(false);
        if (Object.keys(gNodes).length > 0) {
            drawMap(gNodes, gEdges, null, [], null, null, [], null);
        }
    }
}

function showSimControls(visible) {
    const el = document.getElementById('simControls');
    if (el) el.style.display = visible ? 'block' : 'none';
}

// 场地边界检测（模拟器和线上模式共用）
function checkBoundary(pos, stopOnOut = false) {
    const mapData = window.__MAP_DATA__;
    if (!mapData || !mapData.field_size_mm) return true;
    const fieldW = mapData.field_size_mm[0];
    const fieldH = mapData.field_size_mm[1];
    const halfW = fieldW / 2;
    const outOfBounds = Math.abs(pos.x) > halfW || pos.y < 0 || pos.y > fieldH;
    if (outOfBounds) {
        pos.x = Math.max(-halfW, Math.min(halfW, pos.x));
        pos.y = Math.max(0, Math.min(fieldH, pos.y));
        return false;
    }
    return true;
}

// 页面加载完成后自动初始化
window.addEventListener('DOMContentLoaded', initDashboard);
