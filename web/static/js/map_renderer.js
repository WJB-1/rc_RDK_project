// 地图渲染
let mapRenderNodes = {};
let mapRenderEdges = [];

function computeMapTransform(w, h, nodes) {
    if (!nodes || Object.keys(nodes).length === 0) {
        return { scale: 0.1, ox: w / 2, oy: 30 };
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
    const scaleX = availW / (maxX - minX || 1);
    const scaleY = availH / (maxY - minY || 1);
    const scale = Math.min(scaleX, scaleY);
    const ox = w / 2 - (minX + maxX) / 2 * scale;
    const oy = padding + (availH - (maxY - minY) * scale) / 2 - minY * scale;
    return { scale, ox, oy };
}

function renderMap(data) {
    mapRenderNodes = data.map_data ? data.map_data.nodes || {} : gNodes;
    mapRenderEdges = data.map_data ? data.map_data.edges || [] : gEdges;

    const position = data.position || null;
    const visitedNodes = data.visited_nodes || [];
    const currentNode = data.current_node;
    const targetNode = data.target_node;
    const plannedPath = data.planned_path || [];
    const trajectory = data.trajectory || null;

    drawMap(mapRenderNodes, mapRenderEdges, position, visitedNodes,
            currentNode, targetNode, plannedPath, trajectory);
}

function renderMapFromSim() {
    drawMap(
        gNodes, gEdges,
        [sim.pos.x, sim.pos.y, sim.pos.yaw],
        sim.visited,
        sim.currentNode,
        sim.targetNode,
        sim.path,
        sim.trajectory
    );
}

function drawMap(nodes, edges, position, visitedNodes, currentNode, targetNode, plannedPath, trajectory) {
    const canvas = document.getElementById('mapCanvas');
    const ctx = canvas.getContext('2d');
    const dpr = window.devicePixelRatio || 1;
    const rect = canvas.getBoundingClientRect();

    // 只在尺寸变化时重置 canvas，避免每帧闪烁
    const targetW = rect.width * dpr;
    const targetH = rect.height * dpr;
    if (canvas.width !== targetW || canvas.height !== targetH) {
        canvas.width = targetW;
        canvas.height = targetH;
    }
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);

    const w = rect.width;
    const h = rect.height;
    const tf = computeMapTransform(w, h, nodes);
    const s = tf.scale;
    const ox = tf.ox;
    const oy = tf.oy;

    // 缓存变换参数供拖拽使用
    lastMapTransform = { scale: s, ox, oy };

    ctx.fillStyle = '#111';
    ctx.fillRect(0, 0, w, h);

    // 网格
    ctx.strokeStyle = '#1a1a2e';
    ctx.lineWidth = 1;
    for (let x = 0; x < w; x += 50) {
        ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, h); ctx.stroke();
    }
    for (let y = 0; y < h; y += 50) {
        ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(w, y); ctx.stroke();
    }

    function toCanvas(x_mm, y_mm) {
        return { x: ox + x_mm * s, y: oy + y_mm * s };
    }

    // 车道物理宽度对应的像素，默认 200mm
    const laneWidthPx = (window.__MAP_DATA__?.lane_width_mm || 200) * s;
    const wallWidthPx = Math.max(2, 30 * s);
    const totalWidth = laneWidthPx + wallWidthPx * 2;
    const junctionSize = totalWidth;

    // 工具函数
    function strokeRoad(a, b, width, color) {
        const pa = toCanvas(a.x, a.y);
        const pb = toCanvas(b.x, b.y);
        ctx.beginPath();
        ctx.moveTo(pa.x, pa.y);
        ctx.lineTo(pb.x, pb.y);
        ctx.strokeStyle = color;
        ctx.lineWidth = width;
        ctx.lineCap = 'butt';
        ctx.stroke();
    }

    function fillJunction(node, size, color) {
        const p = toCanvas(node.x, node.y);
        ctx.fillStyle = color;
        ctx.fillRect(p.x - size / 2, p.y - size / 2, size, size);
    }

    // === 第1层：围墙底板（所有道路外扩）===
    edges.forEach(edge => {
        const a = nodes[edge.node_a];
        const b = nodes[edge.node_b];
        if (a && b) strokeRoad(a, b, totalWidth, '#252a2d');
    });

    // === 第2层：路口围墙连接块（消除 N1/N12 缺角）===
    Object.values(nodes).forEach(node => {
        fillJunction(node, junctionSize, '#252a2d');
    });

    // === 第3层：路面 ===
    edges.forEach(edge => {
        const a = nodes[edge.node_a];
        const b = nodes[edge.node_b];
        if (a && b) {
            strokeRoad(a, b, laneWidthPx, edge.is_tunnel ? '#8B3A3A' : '#5A5A5A');
        }
    });

    // === 第4层：路口路面块 + 标线 + 隧道标记 ===
    Object.values(nodes).forEach(node => {
        fillJunction(node, laneWidthPx, '#5A5A5A');
    });

    // 标线 & 隧道文字（独立循环）
    edges.forEach(edge => {
        const a = nodes[edge.node_a];
        const b = nodes[edge.node_b];
        if (!a || !b) return;
        const pa = toCanvas(a.x, a.y);
        const pb = toCanvas(b.x, b.y);

        // 道路中心虚线
        ctx.beginPath();
        ctx.moveTo(pa.x, pa.y);
        ctx.lineTo(pb.x, pb.y);
        ctx.strokeStyle = 'rgba(255, 255, 255, 0.15)';
        ctx.lineWidth = 1.5;
        ctx.setLineDash([8, 10]);
        ctx.stroke();
        ctx.setLineDash([]);

        // 隧道标记
        if (edge.is_tunnel) {
            const mx = (pa.x + pb.x) / 2;
            const my = (pa.y + pb.y) / 2;
            ctx.fillStyle = '#ff6b6b';
            ctx.font = 'bold 11px sans-serif';
            ctx.textAlign = 'center';
            ctx.fillText('隧道', mx, my - laneWidthPx / 2 - 8);
        }
    });

    // 节点圆盘半径：半透明小圆，不盖住车道
    const nr = Math.max(4, Math.min(laneWidthPx * 0.3, 10));

    Object.entries(nodes).forEach(([name, node]) => {
        const p = toCanvas(node.x, node.y);
        const isVisited = visitedNodes.includes(name);
        const isCurrent = name === currentNode;
        const isTarget = name === targetNode;
        const isPlanned = plannedPath.includes(name);

        // 半透明颜色
        let color = 'rgba(149, 165, 166, 0.6)';          // 灰色默认
        if (isCurrent) color = 'rgba(231, 76, 60, 0.7)';  // 红
        else if (isTarget) color = 'rgba(243, 156, 18, 0.7)'; // 橙
        else if (isVisited) color = 'rgba(39, 174, 96, 0.7)'; // 绿
        else if (isPlanned) color = 'rgba(52, 152, 219, 0.7)'; // 蓝

        ctx.fillStyle = color;
        ctx.beginPath();
        ctx.arc(p.x, p.y, nr, 0, Math.PI * 2);
        ctx.fill();

        ctx.strokeStyle = 'rgba(255, 255, 255, 0.5)';
        ctx.lineWidth = 1;
        ctx.stroke();

        // 统一字体大小
        const isJunction = name.startsWith('T') || name === 'J_START';
        ctx.fillStyle = '#fff';
        ctx.font = 'bold 11px sans-serif';
        ctx.textAlign = 'center';
        ctx.textBaseline = 'bottom';
        if (!isJunction || s > 0.15) {
            ctx.fillText(name, p.x, p.y - nr - 2);
        }
    });

    // 小车位置
    if (position) {
        const [x, y, yaw] = position;
        const p = toCanvas(x, y);
        const yawRad = ((90 - yaw) * Math.PI) / 180;

        // 小车物理尺寸: 宽 115mm × 长 170mm, 按 scale 换算
        const carW = 115 * s;
        const carL = 170 * s;
        const minCarW = 6, minCarL = 10;
        const halfW = Math.max(minCarW / 2, carW / 2);
        const halfL = Math.max(minCarL / 2, carL / 2);

        // 绘制车体 (圆角矩形, 朝向 yaw)
        ctx.save();
        ctx.translate(p.x, p.y);
        ctx.rotate(yawRad);

        // 车身
        ctx.fillStyle = '#f39c12';
        ctx.strokeStyle = '#fff';
        ctx.lineWidth = 1;
        const r = Math.min(halfW * 0.5, 4);
        ctx.beginPath();
        ctx.moveTo(-halfL + r, -halfW);
        ctx.lineTo(halfL - r, -halfW);
        ctx.arcTo(halfL, -halfW, halfL, -halfW + r, r);
        ctx.lineTo(halfL, halfW - r);
        ctx.arcTo(halfL, halfW, halfL - r, halfW, r);
        ctx.lineTo(-halfL + r, halfW);
        ctx.arcTo(-halfL, halfW, -halfL, halfW - r, r);
        ctx.lineTo(-halfL, -halfW + r);
        ctx.arcTo(-halfL, -halfW, -halfL + r, -halfW, r);
        ctx.closePath();
        ctx.fill();
        ctx.stroke();

        const handleDistance = halfL + 14;
        ctx.beginPath();
        ctx.moveTo(halfL, 0);
        ctx.lineTo(handleDistance, 0);
        ctx.strokeStyle = '#4fc3f7';
        ctx.lineWidth = 2;
        ctx.stroke();
        ctx.fillStyle = '#4fc3f7';
        ctx.beginPath();
        ctx.arc(handleDistance, 0, 5, 0, Math.PI * 2);
        ctx.fill();

        // 车头方向小三角
        ctx.fillStyle = '#e74c3c';
        ctx.beginPath();
        ctx.moveTo(halfL + 2, 0);
        ctx.lineTo(halfL - halfW * 0.4, -halfW * 0.6);
        ctx.lineTo(halfL - halfW * 0.4, halfW * 0.6);
        ctx.closePath();
        ctx.fill();

        ctx.restore();

        // 航向角标注
        ctx.fillStyle = '#f39c12';
        ctx.font = `${Math.max(9, halfL * 0.4)}px sans-serif`;
        ctx.fillText(`${yaw.toFixed(1)}°`, p.x + halfL + 6, p.y + 4);
    }

    // 轨迹
    if (trajectory && trajectory.length > 1) {
        ctx.strokeStyle = 'rgba(46, 204, 113, 0.5)';
        ctx.lineWidth = 2;
        ctx.setLineDash([5, 5]);
        ctx.beginPath();
        trajectory.forEach((pos, i) => {
            const tp = toCanvas(pos[0], pos[1]);
            if (i === 0) ctx.moveTo(tp.x, tp.y);
            else ctx.lineTo(tp.x, tp.y);
        });
        ctx.stroke();
        ctx.setLineDash([]);
    }
}

// 统计
function updateStats() {
    // 纯数据初始化，不依赖 DOM 元素（独立 HTML 可能没有统计面板）
    // 地图数据已在 initDashboard 中加载到 gNodes / gEdges
    const nodeVals = Object.values(gNodes);
    if (nodeVals.length > 0) {
        console.log(`[dashboard] 地图加载: ${nodeVals.length} 节点, ${gEdges.length} 边`);
    }
}
