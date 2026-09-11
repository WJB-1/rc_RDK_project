// 模拟器 — 复用 Python 端 PathPlanner API，不重复实现路径逻辑
let _simPendingPlan = false;   // 防止并发请求
let _simCurrentEdgeSpeed = 300; // mm/s，隧道时降为 150

function toggleSimulation() {
    if (simRunning) { stopSimulation(); }
    else { startSimulation(); }
}

async function startSimulation() {
    const startSelect = document.getElementById('simStartNode');
    const startName = (startSelect && startSelect.value) || 'START';
    const startNode = gNodes[startName];
    if (!startNode) { logCmd('错误: 起始节点不存在', 'error'); return; }

    sim.pos = { x: startNode.x, y: startNode.y, yaw: 0 };
    sim.odom = 0;
    sim.state = 'GLOBAL_PLANNING';
    sim.currentNode = startName;
    sim.visited = [];
    sim.blockedEdges = [];
    sim.trajectory = [[startNode.x, startNode.y]];
    sim.events = [];
    sim.currentEdge = null;

    // 调 Python 端 PathPlanner API 做首次全局规划
    const plan = await fetchSimPlan(startName, sim.visited, sim.blockedEdges);
    if (!plan || !plan.edge_tasks || plan.edge_tasks.length === 0) {
        logCmd('错误: 路径规划失败', 'error');
        return;
    }

    sim.edgeTasks = plan.edge_tasks;
    sim.edgeIndex = 0;
    sim.totalDistance = plan.total_distance_mm;

    // 加载第一条边
    _loadEdge(0);

    if (startNode.type === 'mission') {
        sim.visited.push(startName);
        addSimEvent('start', `从 ${startName} 出发 (已打卡)`);
    } else {
        addSimEvent('start', `从 ${startName} 出发，${plan.edge_tasks.length} 段边，总长 ${plan.total_distance_mm.toFixed(0)}mm`);
    }

    carDragging = false;
    simRunning = true;
    updateSimButton();
    simInterval = setInterval(simTick, 50);
    logCmd(`模拟器已启动 (起点: ${startName})`);
}

async function fetchSimPlan(currentNode, visited, blockedEdges) {
    try {
        const resp = await fetch('/api/sim_plan', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({
                current_node: currentNode,
                visited: visited,
                blocked_edges: blockedEdges,
            })
        });
        if (!resp.ok) return null;
        return await resp.json();
    } catch (e) {
        logCmd(`路径规划API错误: ${e.message}`, 'error');
        return null;
    }
}

function _loadEdge(index) {
    if (index >= sim.edgeTasks.length) {
        // 当前路径走完，请求重规划
        _requestReplan();
        return;
    }
    sim.edgeIndex = index;
    const task = sim.edgeTasks[index];
    sim.currentEdge = task;
    sim.targetNode = task.to_node;
    sim.state = 'EDGE_EXECUTING';
    sim.edgeOdomStart = sim.odom;

    // 沿边直线插值：记录边的几何起点/终点/长度，位置由 edgeDist 精确决定
    const fn = gNodes[task.from_node];
    const tn = gNodes[task.to_node];
    if (fn && tn) {
        sim.edgeFrom = { x: fn.x, y: fn.y };
        sim.edgeTo = { x: tn.x, y: tn.y };
        sim.edgeLen = Math.hypot(tn.x - fn.x, tn.y - fn.y) || task.distance_mm || 1;
    }
    sim.edgeDist = 0;  // 进入新边，已走距离清零

    // 隧道减速
    _simCurrentEdgeSpeed = task.is_tunnel ? 150 : 300;
    addSimEvent('edge_start',
        `${task.from_node}→${task.to_node} ${task.distance_mm.toFixed(0)}mm` +
        (task.is_tunnel ? ' [隧道]' : ''));
}

async function _requestReplan() {
    if (_simPendingPlan) return;
    _simPendingPlan = true;

    const plan = await fetchSimPlan(sim.currentNode, sim.visited, sim.blockedEdges);
    _simPendingPlan = false;

    if (!plan || !plan.edge_tasks || plan.edge_tasks.length === 0) {
        sim.state = 'FINISHED';
        sim.targetNode = null;
        addSimEvent('finished', '所有任务完成');
        stopSimulation();
        return;
    }

    sim.edgeTasks = plan.edge_tasks;
    sim.totalDistance = plan.total_distance_mm;
    _loadEdge(0);
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
    btn.textContent = simRunning ? '停止模拟' : '启动模拟';
    btn.className = simRunning ? 'btn btn-stop' : 'btn btn-action';
}

function simTick() {
    if (!simRunning || !sim.targetNode || !gNodes[sim.targetNode]) return;

    const dt = 0.05;
    const step = _simCurrentEdgeSpeed * dt;

    // 沿当前边直线插值：位置由 edgeDist / edgeLen 精确决定，车始终贴边、不漂移
    if (sim.edgeFrom && sim.edgeTo) {
        sim.edgeDist += step;  // 已走距离单调递增

        const fn = sim.edgeFrom, tn = sim.edgeTo;
        const len = sim.edgeLen;
        const ratio = Math.min(1.0, sim.edgeDist / len);

        sim.pos.x = fn.x + (tn.x - fn.x) * ratio;
        sim.pos.y = fn.y + (tn.y - fn.y) * ratio;

        // yaw 对齐边方向（沿边前进方向）
        const dx = tn.x - fn.x, dy = tn.y - fn.y;
        sim.pos.yaw = (Math.atan2(dx, dy) * 180) / Math.PI;

        // 已走满整条边 → 吸附到精确端点，进入节点处理
        if (ratio >= 1.0) {
            sim.pos.x = tn.x;
            sim.pos.y = tn.y;
            sim.edgeDist = len;
            _handleNodeArrival();
            updateTelemetryFromSim();
            renderMapFromSim();
            return;
        }
    } else {
        // 无边几何信息时退化：朝目标节点直线推进（保证不静止）
        const tgt = gNodes[sim.targetNode];
        const dx = tgt.x - sim.pos.x;
        const dy = tgt.y - sim.pos.y;
        const dist = Math.hypot(dx, dy);
        if (dist < 5) {
            sim.pos.x = tgt.x;
            sim.pos.y = tgt.y;
            _handleNodeArrival();
            updateTelemetryFromSim();
            renderMapFromSim();
            return;
        }
        sim.pos.x += (dx / dist) * step;
        sim.pos.y += (dy / dist) * step;
    }

    sim.odom += step;

    if (typeof checkBoundary === 'function' && !checkBoundary(sim.pos, true)) {
        stopSimulation();
        addSimEvent('error', '小车超出场地边界，模拟已停止');
        logCmd('错误: 小车超出场地边界，模拟已停止', 'error');
        updateTelemetryFromSim(); renderMapFromSim();
        return;
    }

    sim.trajectory.push([sim.pos.x, sim.pos.y]);
    if (sim.trajectory.length > 2000) sim.trajectory = sim.trajectory.slice(-2000);
    updateTelemetryFromSim();
    renderMapFromSim();
}

function _handleNodeArrival() {
    const nodeName = sim.targetNode;
    if (!nodeName || !gNodes[nodeName]) return;

    sim.currentNode = nodeName;

    // mission 节点且未打卡 → RFID 打卡
    if (gNodes[nodeName].rfid && !sim.visited.includes(nodeName)) {
        sim.visited.push(nodeName);
        addSimEvent('rfid_scanned', `打卡: ${nodeName}`);
        sim.log.push(`打卡: ${nodeName}`);
        // 打卡后检查是否需要重规划（blocked_edges 变化场景）
        // 正常打卡不重跑 TSP，只推进到下一条边
    }

    // 推进到下一条边
    _loadEdge(sim.edgeIndex + 1);
}

function addSimEvent(type, message) {
    sim.events.push({ timestamp: Date.now() / 1000, type, message });
    if (sim.events.length > 200) sim.events = sim.events.slice(-200);
    updateEventLog(sim.events);
}
