// 控制面板
let leadDistance = 80;  // 路口提前量 (mm)

function updateSlider(name, value) {
    if (name === 'vx') {
        vxSpeed = parseInt(value);
        document.getElementById('vxValue').textContent = vxSpeed;
    } else if (name === 'lead') {
        leadDistance = parseInt(value);
        document.getElementById('leadValue').textContent = leadDistance;
    }
}

function toggleAutoMode() {
    currentMode = 'auto';
    document.getElementById('btnAutoMode').classList.add('active');
    document.getElementById('btnManualMode').classList.remove('active');
    logCmd('切换到自动模式：启动路径规划 + 50Hz tick');
    sendModeCmd('auto');
}

function setManualMode() {
    currentMode = 'manual';
    document.getElementById('btnAutoMode').classList.remove('active');
    document.getElementById('btnManualMode').classList.add('active');
    logCmd('切换到手动模式：停止状态机 tick');
    sendModeCmd('manual');
}

async function sendModeCmd(mode) {
    if (window.location.protocol === 'file:' || !ws || ws.readyState !== WebSocket.OPEN) {
        logCmd(`[模拟] 模式切换: ${mode}`);
        return;
    }
    try {
        const response = await fetch('/api/mode', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ mode: mode })
        });
        const result = await response.json();
        if (!result.ok) {
            logCmd(`模式切换错误: ${result.error}`, 'error');
        }
    } catch (e) {
        logCmd(`网络错误: ${e.message}`, 'error');
    }
}

function sendCmd(cmd) {
    // 全部离散指令，单击即执行
    const cmdMap = {
        'forward':              { cmd: 'move', vx: vxSpeed, distance: 300 },
        'backward':             { cmd: 'move', vx: -vxSpeed, distance: 300 },
        'turn_left_90':         { cmd: 'turn_imu', angle: 90 },
        'turn_right_90':        { cmd: 'turn_imu', angle: -90 },
        'intersection_left':    { cmd: 'intersection_turn', direction: 'left', lead: leadDistance },
        'intersection_right':   { cmd: 'intersection_turn', direction: 'right', lead: leadDistance },
        'stop':                 { cmd: 'stop' },
        'reset_odom':           { cmd: 'reset_odom' },
    };

    const payload = cmdMap[cmd];
    if (payload) {
        logCmd(`发送: ${cmd} ${JSON.stringify(payload)}`);
        if (typeof sendApiCmd === 'function') {
            sendApiCmd(payload.cmd, payload);
        }
    }
}

async function sendApiCmd(cmd, payload) {
    // 独立模式 (file:// 或无 WebSocket): 仅记录日志
    if (window.location.protocol === 'file:' || !ws || ws.readyState !== WebSocket.OPEN) {
        logCmd(`[模拟] ${cmd}: ${JSON.stringify(payload)}`);
        return;
    }
    try {
        const response = await fetch('/api/cmd', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(payload)
        });
        const result = await response.json();
        if (!result.ok) {
            logCmd(`错误: ${result.error}`, 'error');
        }
    } catch (e) {
        logCmd(`网络错误: ${e.message}`, 'error');
    }
}

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

// 入口
window.addEventListener('resize', () => {
    if (simRunning) {
        renderMapFromSim();
    }
});

// 小车拖拽
function setupCarDrag() {
    if (!canvas) canvas = document.getElementById('mapCanvas');
    if (!canvas) return;

    canvas.addEventListener('mousedown', onCarMouseDown);
    canvas.addEventListener('mousemove', onCarMouseMove);
    canvas.addEventListener('mouseup', onCarMouseUp);
    canvas.addEventListener('mouseleave', onCarMouseUp);
    canvas.addEventListener('touchstart', onCarTouchStart, { passive: false });
    canvas.addEventListener('touchmove', onCarTouchMove, { passive: false });
    canvas.addEventListener('touchend', onCarMouseUp);
}

function getCarRect() {
    // 返回小车在 canvas 上的位置和尺寸
    const tf = lastMapTransform;
    if (!tf || !sim.pos) return null;

    const carW = 115 * tf.scale;
    const carL = 170 * tf.scale;
    const halfW = Math.max(3, carW / 2);
    const halfL = Math.max(5, carL / 2);
    return {
        cx: tf.ox + sim.pos.x * tf.scale,
        cy: tf.oy + sim.pos.y * tf.scale,
        halfW, halfL,
        yawRad: (90 - sim.pos.yaw) * Math.PI / 180,
    };
}

function getRotationHandle() {
    const r = getCarRect();
    if (!r) return null;
    const distance = r.halfL + 14;
    return {
        x: r.cx + Math.cos(r.yawRad) * distance,
        y: r.cy + Math.sin(r.yawRad) * distance,
        radius: 10,
    };
}

function hitTestCar(mx, my) {
    const r = getCarRect();
    if (!r) return false;
    // 将鼠标坐标变换到车体坐标系
    const dx = mx - r.cx;
    const dy = my - r.cy;
    const cos = Math.cos(-r.yawRad);
    const sin = Math.sin(-r.yawRad);
    const lx = dx * cos - dy * sin;  // 车体纵向
    const ly = dx * sin + dy * cos;  // 车体横向
    // 扩展一点命中区域方便手指操作
    const margin = 4;
    return Math.abs(lx) < (r.halfL + margin) && Math.abs(ly) < (r.halfW + margin);
}

function hitTestRotationHandle(mx, my) {
    const handle = getRotationHandle();
    return handle && Math.hypot(mx - handle.x, my - handle.y) <= handle.radius;
}

function updateCarYaw(mx, my) {
    const r = getCarRect();
    if (!r) return;
    const canvasAngle = Math.atan2(my - r.cy, mx - r.cx) * 180 / Math.PI;
    sim.pos.yaw = ((90 - canvasAngle + 540) % 360) - 180;
}

function canvasToWorld(mx, my) {
    const tf = lastMapTransform;
    if (!tf) return null;
    return {
        x: (mx - tf.ox) / tf.scale,
        y: (my - tf.oy) / tf.scale,
    };
}

function onCarMouseDown(e) {
    if (simRunning) return;  // 运行中不允许拖拽
    const rect = e.target.getBoundingClientRect();
    const mx = e.clientX - rect.left;
    const my = e.clientY - rect.top;
    if (hitTestRotationHandle(mx, my)) {
        e.preventDefault();
        carDragging = true;
        carDragMode = 'rotate';
        carPosBeforeDrag = { ...sim.pos };
        carDragStart = { mx, my };
        canvas.style.cursor = 'grabbing';
    } else if (hitTestCar(mx, my)) {
        e.preventDefault();
        carDragging = true;
        carDragMode = 'move';
        carPosBeforeDrag = { ...sim.pos };
        carDragStart = { mx, my };
        canvas.style.cursor = 'grabbing';
    }
}

function onCarMouseMove(e) {
    const rect = e.target.getBoundingClientRect();
    const mx = e.clientX - rect.left;
    const my = e.clientY - rect.top;

    if (carDragging) {
        if (carDragMode === 'rotate') {
            updateCarYaw(mx, my);
            updateTelemetryFromSim();
            renderMapFromSim();
        } else {
            const world = canvasToWorld(mx, my);
            if (!world) return;
            sim.pos.x = world.x;
            sim.pos.y = world.y;
            snapToNearestNode();
            updateTelemetryFromSim();
            renderMapFromSim();
        }
        return;
    }

    // 悬停检测
    if (!simRunning && (hitTestCar(mx, my) || hitTestRotationHandle(mx, my))) {
        canvas.style.cursor = 'grab';
    } else if (!carDragging) {
        canvas.style.cursor = '';
    }
}

function onCarMouseUp(e) {
    if (!carDragging) return;
    carDragging = false;
    const wasRotating = carDragMode === 'rotate';
    carDragMode = null;
    canvas.style.cursor = '';
    snapToNearestNode();
    updateTelemetryFromSim();
    renderMapFromSim();

    if (!wasRotating && sim.currentNode) {
        const sel = document.getElementById('simStartNode');
        if (sel) sel.value = sim.currentNode;
    }
    logCmd(wasRotating
        ? `小车航向已调整为 ${sim.pos.yaw.toFixed(1)}°`
        : `小车已移至 ${sim.currentNode} (${sim.pos.x.toFixed(0)}, ${sim.pos.y.toFixed(0)})`);
}

function onCarTouchStart(e) {
    if (simRunning || e.touches.length !== 1) return;
    const rect = e.target.getBoundingClientRect();
    const mx = e.touches[0].clientX - rect.left;
    const my = e.touches[0].clientY - rect.top;
    if (hitTestRotationHandle(mx, my)) {
        e.preventDefault();
        carDragging = true;
        carDragMode = 'rotate';
        carPosBeforeDrag = { ...sim.pos };
    } else if (hitTestCar(mx, my)) {
        e.preventDefault();
        carDragging = true;
        carDragMode = 'move';
        carPosBeforeDrag = { ...sim.pos };
    }
}

function onCarTouchMove(e) {
    if (!carDragging || e.touches.length !== 1) return;
    e.preventDefault();
    const rect = e.target.getBoundingClientRect();
    const mx = e.touches[0].clientX - rect.left;
    const my = e.touches[0].clientY - rect.top;
    if (carDragMode === 'rotate') {
        updateCarYaw(mx, my);
        updateTelemetryFromSim();
        renderMapFromSim();
    } else {
        const world = canvasToWorld(mx, my);
        if (!world) return;
        sim.pos.x = world.x;
        sim.pos.y = world.y;
        snapToNearestNode();
        updateTelemetryFromSim();
        renderMapFromSim();
    }
}

function snapToNearestNode() {
    let bestDist = Infinity;
    let bestNode = null;
    for (const [name, node] of Object.entries(gNodes)) {
        const d = Math.hypot(sim.pos.x - node.x, sim.pos.y - node.y);
        if (d < bestDist) {
            bestDist = d;
            bestNode = name;
        }
    }
    if (bestNode && bestDist < 400) {  // 400mm 内自动吸附
        sim.pos.x = gNodes[bestNode].x;
        sim.pos.y = gNodes[bestNode].y;
        sim.currentNode = bestNode;
    }
}
