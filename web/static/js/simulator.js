// 模拟器（独立模式）
function toggleSimulation() {
    if (simRunning) {
        stopSimulation();
    } else {
        startSimulation();
    }
}

function startSimulation() {
    // 读取用户选择的起始节点
    const startSelect = document.getElementById('simStartNode');
    const startName = (startSelect && startSelect.value) || 'START';
    const startNode = gNodes[startName];
    if (!startNode) {
        logCmd('错误: 起始节点不存在', 'error');
        return;
    }

    sim.pos = { x: startNode.x, y: startNode.y, yaw: 0 };
    sim.odom = 0;
    sim.state = 'CRUISING';
    sim.currentNode = startName;
    sim.visited = [];
    sim.pathIndex = 0;
    sim.trajectory = [[startNode.x, startNode.y]];
    sim.events = [];
    sim.approachingUntil = 0;

    // 重新计算从起始节点出发的巡逻路径
    const missionNodes = Object.keys(gNodes).filter(
        n => gNodes[n].type === 'mission'
    );
    // 简化: 用数据中预存的巡逻路径, 从 startName 开始截取
    const fullPath = window.__MAP_DATA__?.patrol_path || [];
    const startIdx = fullPath.indexOf(startName);
    if (startIdx >= 0) {
        sim.path = fullPath.slice(startIdx);
    } else {
        sim.path = fullPath;
    }
    sim.targetNode = sim.path.length > 1 ? sim.path[1] : null;

    // 如果起始是 mission 节点，直接标记已访问
    if (startNode.type === 'mission') {
        sim.visited.push(startName);
        addSimEvent('start', `从 ${startName} 出发 (已打卡)`);
    } else {
        addSimEvent('start', `从 ${startName} 出发`);
    }

    carDragging = false;  // 清除拖拽状态
    simRunning = true;
    updateSimButton();
    simInterval = setInterval(simTick, 50);
    logCmd(`模拟器已启动 (起点: ${startName})`);
}

function stopSimulation() {
    simRunning = false;
    if (simInterval) { clearInterval(simInterval); simInterval = null; }
    updateSimButton();
    logCmd('模拟器已停止');
}

function updateSimButton() {
    const btn = document.getElementById('btnSim');
    if (!btn) return;
    if (simRunning) {
        btn.textContent = '停止模拟';
        btn.className = 'btn btn-stop';
    } else {
        btn.textContent = '启动模拟';
        btn.className = 'btn btn-action';
    }
}

function simTick() {
    if (!simRunning) return;
    if (!sim.targetNode || !gNodes[sim.targetNode]) return;

    const tgt = gNodes[sim.targetNode];
    const dt = 0.05;          // 50ms tick
    const speed = 300;        // mm/s
    const step = speed * dt;  // 15mm per tick

    const dx = tgt.x - sim.pos.x;
    const dy = tgt.y - sim.pos.y;
    const dist = Math.hypot(dx, dy);

    if (dist < 5) {
        // 精确到达目标节点
        sim.pos.x = tgt.x;
        sim.pos.y = tgt.y;
        handleNodeArrival();
        updateTelemetryFromSim();
        renderMapFromSim();
        return;
    }

    // 移动：先横后纵（模拟车道约束走直角弯）
    const ax = Math.abs(dx), ay = Math.abs(dy);
    let moveX = 0, moveY = 0;
    if (ax > ay) {
        moveX = Math.sign(dx) * Math.min(step, ax);
        sim.pos.yaw = dx > 0 ? 90 : -90;
    } else {
        moveY = Math.sign(dy) * Math.min(step, ay);
        sim.pos.yaw = dy > 0 ? 0 : 180;
    }

    sim.pos.x += moveX;
    sim.pos.y += moveY;
    sim.odom += Math.abs(moveX) + Math.abs(moveY);

    // 边界碰撞检测
    if (!checkBoundary(sim.pos, /* stopOnOut */ true)) {
        stopSimulation();
        addSimEvent('error', `小车超出场地边界，模拟已停止`);
        logCmd('错误: 小车超出场地边界，模拟已停止', 'error');
        updateTelemetryFromSim();
        renderMapFromSim();
        return;
    }

    // 保存轨迹
    sim.trajectory.push([sim.pos.x, sim.pos.y]);
    if (sim.trajectory.length > 2000) sim.trajectory = sim.trajectory.slice(-2000);

    updateTelemetryFromSim();
    renderMapFromSim();
}

function handleNodeArrival() {
    const nodeName = sim.targetNode;
    if (!nodeName || !gNodes[nodeName]) return;

    // RFID 打卡
    if (gNodes[nodeName].rfid && !sim.visited.includes(nodeName)) {
        sim.visited.push(nodeName);
        sim.currentNode = nodeName;
        addSimEvent('rfid_scanned', `RFID: ${nodeName} → 坐标硬吸附 → (${sim.pos.x.toFixed(0)}, ${sim.pos.y.toFixed(0)})`);
    }

    // 推进路径
    advanceToNextNode();
}

function advanceToNextNode() {
    const path = sim.path;
    if (!path || path.length === 0) {
        sim.targetNode = null;
        return;
    }

    const idx = path.indexOf(sim.targetNode);
    if (idx < 0 || idx >= path.length - 1) {
        // 路径走完，重新规划
        const unvisited = Object.keys(gNodes).filter(
            n => gNodes[n].type === 'mission' && !sim.visited.includes(n)
        );
        if (unvisited.length === 0) {
            sim.state = 'FINISHED';
            sim.targetNode = null;
            addSimEvent('finished', '所有任务完成');
            stopSimulation();
            return;
        }
        // 简化 TSP：取第一个未访问的 mission 节点
        sim.pathIndex = path.indexOf(unvisited[0]);
        if (sim.pathIndex < 0) sim.pathIndex = 0;
    } else {
        sim.pathIndex = idx + 1;
    }

    if (sim.pathIndex < path.length) {
        sim.targetNode = path[sim.pathIndex];
        sim.currentNode = sim.targetNode;
        sim.state = 'CRUISING';
    }
}

function addSimEvent(type, message) {
    sim.events.push({
        timestamp: Date.now() / 1000,
        type: type,
        message: message,
    });
    if (sim.events.length > 200) sim.events = sim.events.slice(-200);
    updateEventLog(sim.events);
}
