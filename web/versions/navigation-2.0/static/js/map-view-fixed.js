(() => {
  'use strict';
  const NS = 'http://www.w3.org/2000/svg';
  const make = (name, attrs) => { const node = document.createElementNS(NS, name); Object.entries(attrs).forEach(([key, value]) => node.setAttribute(key, value)); return node; };
  window.renderMap = (payload) => {
    const root = document.getElementById('map'); if (!root) return; root.replaceChildren();
    const map = (payload.navigation || {}).map || {}, nodes = Array.isArray(map.nodes) ? map.nodes : [], edges = Array.isArray(map.edges) ? map.edges : [];
    const byId = new Map(nodes.map((node) => [node.node_id || node.id, node])); const points = nodes.filter((node) => Number.isFinite(Number(node.x_mm)) && Number.isFinite(Number(node.y_mm))); if (!points.length) return;
    const xs = points.map((node) => Number(node.x_mm)), ys = points.map((node) => Number(node.y_mm)), minX = Math.min(...xs), maxX = Math.max(...xs), minY = Math.min(...ys), maxY = Math.max(...ys), pad = 70, w = 1000, h = 700, scale = Math.min((w - pad * 2) / Math.max(1, maxX - minX), (h - pad * 2) / Math.max(1, maxY - minY));
    const point = (node) => ({ x: pad + (Number(node.x_mm) - minX) * scale, y: h - pad - (Number(node.y_mm) - minY) * scale }); const group = make('g', { 'stroke-linecap': 'round' });
    edges.forEach((edge) => { const a = byId.get(edge.from_node_id || edge.node_a), b = byId.get(edge.to_node_id || edge.node_b); if (!a || !b) return; const pa = point(a), pb = point(b); group.append(make('line', { x1: pa.x, y1: pa.y, x2: pb.x, y2: pb.y, class: edge.road_kind === 'INTERNAL' ? 'edge' : 'edge clear' })); });
    points.forEach((node) => { const id = node.node_id || node.id, p = point(node); group.append(make('circle', { cx: p.x, cy: p.y, r: node.node_kind === 'port' ? 3 : 8, class: 'node' })); if (node.node_kind !== 'port') { const label = make('text', { x: p.x, y: p.y - 12, 'text-anchor': 'middle', fill: '#cbd5e1', 'font-size': 11 }); label.textContent = id; group.append(label); } });
    const pose = (payload.world || {}).pose || {}; if (pose.x_mm != null && pose.y_mm != null) { const p = point({ x_mm: pose.x_mm, y_mm: pose.y_mm }); group.append(make('circle', { cx: p.x, cy: p.y, r: 11, class: 'robot' })); } root.append(group);
  };
})();
