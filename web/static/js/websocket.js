// WebSocket 连接（线上模式）
let ws = null;
let lastFrameTime = Date.now();
let frameCount = 0;
let fps = 0;

function connectWebSocket() {
    let fallbackTimer = null;
    try {
        ws = new WebSocket(`ws://${window.location.host}/ws`);
        ws.onopen = () => {
            if (fallbackTimer) clearTimeout(fallbackTimer);
            console.log('WebSocket 已连接');
            document.getElementById('statusDot').classList.add('connected');
        };
        ws.onclose = () => {
            console.log('WebSocket 已断开');
            document.getElementById('statusDot').classList.remove('connected');
        };
        ws.onerror = () => {
            console.warn('WebSocket 连接错误');
        };
        ws.onmessage = onWsMessage;
        // 2 秒内连不上就降级到模拟器模式
        fallbackTimer = setTimeout(() => {
            if (ws && ws.readyState !== WebSocket.OPEN) {
                console.warn('WebSocket 超时，切换到独立模式');
                ws.close();
                ws = null;
                switchToStandalone();
            }
        }, 2000);
    } catch (e) {
        console.warn('WebSocket 连接失败，切换到独立模式');
        switchToStandalone();
    }
}

function switchToStandalone() {
    document.getElementById('statusDot').classList.add('connected');
    document.getElementById('fpsDisplay').textContent = '独立模式 | 模拟器就绪';
    showSimControls(true);
    updateTelemetryFromSim();
    renderMapFromSim();
}

function onWsMessage(event) {
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

    // 同步小车位置到 sim.pos（拖拽期间不覆盖）
    if (data.position && !carDragging) {
        sim.pos.x = data.position[0];
        sim.pos.y = data.position[1];
        sim.pos.yaw = data.position[2];

        // 边界检测（线上模式）
        if (!checkBoundary(sim.pos)) {
            logCmd('⚠ 小车超出场地边界！请检查里程计/视觉定位', 'error');
        }
    }

    // 更新地图数据（首次或变化时）
    if (data.map_data && data.map_data.nodes) {
        gNodes = data.map_data.nodes;
        gEdges = data.map_data.edges || [];
    }

    // 只要有位置或地图数据就渲染（小车 + 轨迹不依赖 map_data）
    if (data.position || data.map_data) {
        renderMap(data);
    }

    updateTelemetry(data);

    if (data.planned_path && data.planned_path.length > 0) {
        updatePath(data);
    }

    if (data.events) {
        updateEventLog(data.events);
    }
}
