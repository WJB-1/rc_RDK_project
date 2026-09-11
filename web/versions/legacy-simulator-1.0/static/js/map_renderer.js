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

    // === 视角选择：上帝视角(全图) vs 小车视角(position 为中心放大) ===
    let tf;
    if (window.__cameraMode__ === 'car' && position) {
        // 小车视角：以 position 为中心，放大显示，让前行探测区(visible_range=400mm)铺满
        // 让 800mm（一条直道长度）约等于画布短边的 1/3，即短边能容下 ~2400mm
        const shortSide = Math.min(w, h);
        const carScale = shortSide / 3 / 800;   // 800mm ≈ 短边 1/3
        tf = {
            scale: carScale,
            ox: w / 2 - position[0] * carScale,
            oy: h / 2 - position[1] * carScale,
        };
    } else {
        tf = computeMapTransform(w, h, nodes);
    }
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

    // 车道物理宽度对应的像素，默认 200mm（Block 方块边长也是 200mm）
    const laneWidthPx = (window.__MAP_DATA__?.lane_width_mm || 200) * s;
    const wallWidthPx = Math.max(2, 30 * s);
    const totalWidth = laneWidthPx + wallWidthPx * 2;
    // 方块边长像素（端口模型积木方块 = 200×200mm）
    const blockSizePx = laneWidthPx;

    // 是否为"方块中心"节点（type != 'port' 且 type != 'base'）
    // 端口模型：junction-T / junction-cross / corner 是方块中心，port 是方块边缘点。
    function isBlockNode(type) {
        return type !== 'port' && type !== 'base';
    }

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

    function fillBlock(node, size, color) {
        const p = toCanvas(node.x, node.y);
        ctx.fillStyle = color;
        ctx.fillRect(p.x - size / 2, p.y - size / 2, size, size);
    }

    // === 第1层：围墙底板（仅直道/桥，即 !is_internal）===
    // 内部半边（center-port，is_internal=True）是方块内部的连接，不画车道。
    edges.forEach(edge => {
        if (edge.is_internal) return;
        const a = nodes[edge.node_a];
        const b = nodes[edge.node_b];
        if (a && b) strokeRoad(a, b, totalWidth, '#252a2d');
    });

    // === 第2层：方块围墙底板（只画方块中心节点）===
    // port 节点不画方块——它们是方块边缘点，方块本身由中心节点一笔画出。
    Object.values(nodes).forEach(node => {
        if (!isBlockNode(node.type)) return;
        fillBlock(node, blockSizePx, '#252a2d');
    });

    // === 第3层：路面（仅直道/桥，即 !is_internal）===
    edges.forEach(edge => {
        if (edge.is_internal) return;
        const a = nodes[edge.node_a];
        const b = nodes[edge.node_b];
        if (a && b) {
            let roadColor = '#5A5A5A';
            if (edge.is_tunnel) {
                roadColor = '#8B3A3A';
            }
            if (edge.is_blocked) {
                roadColor = '#8B1A1A';  // 深红 = 已封锁
            } else if (edge.is_reconned) {
                roadColor = '#1F8A4C';  // 绿 = 已探索（侦查完成）
            } else if (edge.has_culvert) {
                roadColor = '#1A5A8A';  // 蓝 = 已发现，未探索
            }
            strokeRoad(a, b, laneWidthPx, roadColor);
        }
    });

    // === 第4层：方块路面块（只画方块中心节点）+ 标线 + 隧道标记 ===
    Object.values(nodes).forEach(node => {
        if (!isBlockNode(node.type)) return;
        fillBlock(node, blockSizePx, '#5A5A5A');
    });

    // 标线 & 隧道文字（独立循环，仅直道/桥）
    edges.forEach(edge => {
        if (edge.is_internal) return;
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
        // port 节点不画圆点——它们是方块边缘点，非可停靠/可标记节点。
        if (node.type === 'port') return;
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

    // === 端口高亮：hover 方块中心时，高亮该方块用到的所有端口 ===
    // 端口坐标 = 方块中心 ± 100mm（内部半边长度），画在方块边缘。
    if (window.__hoverBlockName) {
        const hoveredBlock = nodes[window.__hoverBlockName];
        if (hoveredBlock && isBlockNode(hoveredBlock.type)) {
            // 找出所有连接到该方块中心的 port 节点
            const blockRect = toCanvas(hoveredBlock.x, hoveredBlock.y);
            const portSize = Math.max(6, blockSizePx * 0.15);
            edges.forEach(edge => {
                if (!edge.is_internal) return;   // 端口连接靠内部半边
                const aName = edge.node_a, bName = edge.node_b;
                // 一端是 hoveredBlock，另一端是 port
                let portNode = null;
                if (aName === window.__hoverBlockName) portNode = nodes[bName];
                else if (bName === window.__hoverBlockName) portNode = nodes[aName];
                if (!portNode || portNode.type !== 'port') return;
                const pp = toCanvas(portNode.x, portNode.y);
                ctx.fillStyle = '#ffd700';  // 金色高亮小方块
                ctx.strokeStyle = '#fff';
                ctx.lineWidth = 1;
                ctx.fillRect(pp.x - portSize / 2, pp.y - portSize / 2, portSize, portSize);
                ctx.strokeRect(pp.x - portSize / 2, pp.y - portSize / 2, portSize, portSize);
            });
        }
    }

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

    // === 地面真相覆盖层（仅在启用时绘制） ===
    // 数据格式: window.__simGroundTruthData__ 包含:
    //   { hidden_obstacles: [{edge_id, offset_mm}], hidden_culverts: [{edge_id, offset_mm}],
    //     found_obstacles: [{edge_id, offset_mm}], found_culverts: [{edge_id, offset_mm}] }
    // 命中收集：障碍/涵洞的矩形在这里收集，供交互层 hover/click 使用。
    // 收集放在此 if 块内部，确保只有"当前正在画"的（即当前视角可见的）才被收集，
    // 避免小车视角 hover 到不可见的 hidden 障碍/涵洞。
    const hitObstacles = [];
    const hitCulverts = [];
    if (window.__simShowGroundTruth__ && window.__simGroundTruthData__) {
        const gt = window.__simGroundTruthData__;
        const laneW = (window.__MAP_DATA__?.lane_width_mm || 200);

        // 障碍物: 180mm(横跨车道) × 50mm(沿边)，横在路中间的长条
        const obsAcross = 180 * s;  // 跨度方向 (px)
        const obsAlong  = 50 * s;   // 沿边方向 (px)

        // 涵洞: 车道宽(横跨) × 200mm(沿边)，贴在路侧与车道齐宽
        const culAcross = laneWidthPx; // 跨度方向 (px)
        const culAlong  = 200 * s;     // 沿边方向 (px)

        // 在边上根据 offset 画矩形
        // acrossEdge: 横跨车道的像素尺寸, alongEdge: 沿边方向的像素尺寸
        function drawOnEdge(a, b, offsetMm, alongEdge, acrossEdge, color, alpha, strokeW) {
            const dx = b.x - a.x;
            const dy = b.y - a.y;
            const dist = Math.hypot(dx, dy);
            if (dist < 1) return;
            const ratio = offsetMm / dist;
            const cx = ox + (a.x + dx * ratio) * s;
            const cy = oy + (a.y + dy * ratio) * s;

            ctx.save();
            ctx.fillStyle = color;
            ctx.globalAlpha = alpha;
            if (Math.abs(dx) > Math.abs(dy)) {
                // 水平边: 宽=alongEdge(沿X), 高=acrossEdge(跨Y)
                ctx.fillRect(cx - alongEdge / 2, cy - acrossEdge / 2, alongEdge, acrossEdge);
                if (strokeW > 0) {
                    ctx.strokeStyle = color;
                    ctx.globalAlpha = Math.min(1, alpha + 0.2);
                    ctx.lineWidth = strokeW;
                    ctx.strokeRect(cx - alongEdge / 2, cy - acrossEdge / 2, alongEdge, acrossEdge);
                }
            } else {
                // 垂直边: 宽=acrossEdge(跨X), 高=alongEdge(沿Y)
                ctx.fillRect(cx - acrossEdge / 2, cy - alongEdge / 2, acrossEdge, alongEdge);
                if (strokeW > 0) {
                    ctx.strokeStyle = color;
                    ctx.globalAlpha = Math.min(1, alpha + 0.2);
                    ctx.lineWidth = strokeW;
                    ctx.strokeRect(cx - acrossEdge / 2, cy - alongEdge / 2, acrossEdge, alongEdge);
                }
            }
            ctx.restore();
            return { x: cx, y: cy };
        }

        // 构建边查找表
        const edgeMap = {};
        edges.forEach(function(e) { edgeMap[e.edge_id] = e; });

        // --- 未发现的障碍物（半透明红色，180×50mm横放路中） ---
        if (gt.hidden_obstacles && gt.hidden_obstacles.length > 0) {
            gt.hidden_obstacles.forEach(function(item) {
                const edge = edgeMap[item.edge_id];
                if (!edge) return;
                const a = nodes[edge.node_a], b = nodes[edge.node_b];
                if (!a || !b) return;
                const off = item.offset_mm || edge.distance_mm / 2;
                const pos = drawOnEdge(a, b, off, obsAlong, obsAcross, 'rgba(255,60,60,1)', 0.5, 1.5);
                if (pos) hitObstacles.push({
                    edge_id: item.edge_id, offset_mm: off,
                    cx: pos.x, cy: pos.y,
                    halfW: (Math.abs(b.x - a.x) > Math.abs(b.y - a.y)) ? obsAlong / 2 : obsAcross / 2,
                    halfH: (Math.abs(b.x - a.x) > Math.abs(b.y - a.y)) ? obsAcross / 2 : obsAlong / 2,
                    isHorizontal: Math.abs(b.x - a.x) > Math.abs(b.y - a.y),
                    discovered: false,
                });
            });
        }

        // --- 已发现的障碍物（实心红色 + 白色X，发现后叉掉） ---
        if (gt.found_obstacles && gt.found_obstacles.length > 0) {
            gt.found_obstacles.forEach(function(item) {
                const edge = edgeMap[item.edge_id];
                if (!edge) return;
                const a = nodes[edge.node_a], b = nodes[edge.node_b];
                if (!a || !b) return;
                const off = item.offset_mm || edge.distance_mm / 2;
                const pos = drawOnEdge(a, b, off, obsAlong, obsAcross, 'rgba(255,60,60,1)', 0.9, 1.5);
                if (pos) {
                    const isHoriz = Math.abs(b.x - a.x) > Math.abs(b.y - a.y);
                    hitObstacles.push({
                        edge_id: item.edge_id, offset_mm: off,
                        cx: pos.x, cy: pos.y,
                        halfW: isHoriz ? obsAlong / 2 : obsAcross / 2,
                        halfH: isHoriz ? obsAcross / 2 : obsAlong / 2,
                        isHorizontal: isHoriz,
                        discovered: true,
                    });
                    const xs = obsAcross * 0.5;
                    ctx.save();
                    ctx.translate(pos.x, pos.y);
                    ctx.strokeStyle = '#fff';
                    ctx.lineWidth = 2;
                    ctx.beginPath();
                    ctx.moveTo(-xs, -xs); ctx.lineTo(xs, xs);
                    ctx.moveTo(xs, -xs); ctx.lineTo(-xs, xs);
                    ctx.stroke();
                    ctx.restore();
                }
            });
        }

        // --- 未发现的涵洞（半透明蓝色，车道宽×200mm贴路边） ---
        if (gt.hidden_culverts && gt.hidden_culverts.length > 0) {
            gt.hidden_culverts.forEach(function(item) {
                const edge = edgeMap[item.edge_id];
                if (!edge) return;
                const a = nodes[edge.node_a], b = nodes[edge.node_b];
                if (!a || !b) return;
                const off = item.offset_mm || edge.distance_mm / 2;
                const pos = drawOnEdge(a, b, off, culAlong, culAcross, 'rgba(60,140,220,1)', 0.5, 1);
                if (pos) {
                    const isHoriz = Math.abs(b.x - a.x) > Math.abs(b.y - a.y);
                    hitCulverts.push({
                        edge_id: item.edge_id, offset_mm: off,
                        cx: pos.x, cy: pos.y,
                        halfW: isHoriz ? culAlong / 2 : culAcross / 2,
                        halfH: isHoriz ? culAcross / 2 : culAlong / 2,
                        isHorizontal: isHoriz,
                        discovered: false, recon: false,
                    });
                }
            });
        }

        // --- 已发现的涵洞（实心蓝色） ---
        if (gt.found_culverts && gt.found_culverts.length > 0) {
            gt.found_culverts.forEach(function(item) {
                const edge = edgeMap[item.edge_id];
                if (!edge) return;
                const a = nodes[edge.node_a], b = nodes[edge.node_b];
                if (!a || !b) return;
                const off = item.offset_mm || edge.distance_mm / 2;
                const pos = drawOnEdge(a, b, off, culAlong, culAcross, 'rgba(60,140,220,1)', 0.9, 1);
                if (pos) {
                    const isHoriz = Math.abs(b.x - a.x) > Math.abs(b.y - a.y);
                    hitCulverts.push({
                        edge_id: item.edge_id, offset_mm: off,
                        cx: pos.x, cy: pos.y,
                        halfW: isHoriz ? culAlong / 2 : culAcross / 2,
                        halfH: isHoriz ? culAcross / 2 : culAlong / 2,
                        isHorizontal: isHoriz,
                        discovered: true,
                        recon: gt.recon_culverts && gt.recon_culverts.indexOf(item.edge_id) !== -1,
                    });
                }
            });
        }
    }

    // === 端口高亮（hover 方块中心时，画该方块的端口）===
    // 端口默认隐藏；hover 某方块时用黄色小方块在边缘高亮展示
    if (window.__hoverBlockName__) {
        Object.entries(nodes).forEach(([name, node]) => {
            if (node.type !== 'port') return;
            // 端口名格式: {方块名}.P_{方向}
            const blockName = name.split('.P_')[0];
            if (blockName !== window.__hoverBlockName__) return;
            const p = toCanvas(node.x, node.y);
            const portSize = Math.max(4, laneWidthPx * 0.2);
            ctx.fillStyle = '#f1c40f';
            ctx.strokeStyle = '#fff';
            ctx.lineWidth = 1;
            ctx.fillRect(p.x - portSize / 2, p.y - portSize / 2, portSize, portSize);
            ctx.strokeRect(p.x - portSize / 2, p.y - portSize / 2, portSize, portSize);
            // 端口方向标注
            const dir = name.split('.P_')[1];
            ctx.fillStyle = '#f1c40f';
            ctx.font = 'bold 10px sans-serif';
            ctx.textAlign = 'center';
            ctx.textBaseline = 'bottom';
            ctx.fillText(dir, p.x, p.y - portSize);
        });
    }

    // === 命中区域收集（供交互层 hover/click 使用）===
    // 记录方块中心矩形、直道线段、端口位置，存到全局 window.__hitRegions
    const hitBlocks = [];
    const hitEdges = [];
    const hitPorts = [];

    Object.entries(nodes).forEach(([name, node]) => {
        const p = toCanvas(node.x, node.y);
        if (node.type === 'port') {
            hitPorts.push({
                name: name,
                blockName: name.split('.P_')[0],
                dir: name.split('.P_')[1] || '',
                px: p.x, py: p.y,
                node: node,
            });
        } else if (node.type !== 'base') {
            // 方块中心：矩形命中区域
            const bs = blockSizePx;
            hitBlocks.push({
                name: name,
                px: p.x, py: p.y,
                half: bs / 2,
                node: node,
            });
        } else {
            // START base：小圆点命中
            hitBlocks.push({
                name: name,
                px: p.x, py: p.y,
                half: Math.max(6, laneWidthPx * 0.4),
                node: node,
            });
        }
    });

    edges.forEach(edge => {
        if (edge.is_internal) return;
        const a = nodes[edge.node_a];
        const b = nodes[edge.node_b];
        if (!a || !b) return;
        const pa = toCanvas(a.x, a.y);
        const pb = toCanvas(b.x, b.y);
        hitEdges.push({
            edge_id: edge.edge_id,
            ax: pa.x, ay: pa.y,
            bx: pb.x, by: pb.y,
            half: laneWidthPx / 2,
            edge: edge,
        });
    });

    window.__hitRegions = {
        blocks: hitBlocks,
        edges: hitEdges,
        ports: hitPorts,
        obstacles: hitObstacles,
        culverts: hitCulverts,
        scale: s, ox: ox, oy: oy,
    };

    // === 选中高亮：对 window.__selectedEntity__ 画亮边框 ===
    if (window.__selectedEntity__) {
        const sel = window.__selectedEntity__;
        if (sel.type === 'obstacle') {
            ctx.strokeStyle = '#ffff00';   // 亮黄
            ctx.lineWidth = 3;
            ctx.strokeRect(sel.cx - sel.halfW, sel.cy - sel.halfH, sel.halfW * 2, sel.halfH * 2);
        } else if (sel.type === 'culvert') {
            ctx.strokeStyle = '#00e5ff';   // 亮青
            ctx.lineWidth = 3;
            ctx.strokeRect(sel.cx - sel.halfW, sel.cy - sel.halfH, sel.halfW * 2, sel.halfH * 2);
        }
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
