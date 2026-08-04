// 遥测数据更新
function updateTelemetry(data) {
    document.getElementById('agentState').textContent = data.agent_state || '--';
    document.getElementById('agentState').className = 'value ' +
        (data.agent_state === 'FINISHED' ? 'ok' :
         data.agent_state === 'TURNING' ? 'warn' : '');

    if (data.position) {
        const [x, y, yaw] = data.position;
        document.getElementById('position').textContent = `${x.toFixed(0)}, ${y.toFixed(0)}`;
        document.getElementById('yaw').textContent = `${yaw.toFixed(1)}°`;
    }

    document.getElementById('currentNode').textContent = data.current_node || '--';
    document.getElementById('targetNode').textContent = data.target_node || '--';

    const offsetEl = document.getElementById('offsetMm');
    const off = data.offset_mm || 0;
    offsetEl.textContent = data.offset_mm !== undefined ? `${off.toFixed(1)} mm` : '--';
    offsetEl.className = 'value ' +
        (Math.abs(off) > 100 ? 'alert' : Math.abs(off) > 50 ? 'warn' : 'ok');

    const interEl = document.getElementById('intersection');
    interEl.textContent = data.is_intersection ? '检测到路口 ⚠' : '正常';
    interEl.className = 'value ' + (data.is_intersection ? 'warn' : 'ok');

    document.getElementById('progress').textContent =
        data.progress ? `${data.progress.visited}/${data.progress.total}` : '--';
}

function updateTelemetryFromSim() {
    updateTelemetry({
        agent_state: sim.state,
        position: [sim.pos.x, sim.pos.y, sim.pos.yaw],
        current_node: sim.currentNode,
        target_node: sim.targetNode,
        offset_mm: (Math.random() - 0.5) * 20,           // 模拟 ±10mm 小幅偏移
        is_intersection: sim.state === 'APPROACHING',
        progress: { visited: sim.visited.length, total: 12 },
        planned_path: sim.path,
        visited_nodes: sim.visited,
    });
}

// 路径显示
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

// 事件日志
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
