(function () {
  "use strict";

  var laneWidthMm = 200;
  var vehicleLengthMm = 170;
  var vehicleWidthMm = 115;

  function transformFor(canvas, nodes) {
    var visible = nodes.filter(function (node) { return node.node_kind !== "port"; });
    var xs = visible.map(function (node) { return node.x_mm; });
    var ys = visible.map(function (node) { return node.y_mm; });
    var minX = Math.min.apply(null, xs) - laneWidthMm;
    var maxX = Math.max.apply(null, xs) + laneWidthMm;
    var minY = Math.min.apply(null, ys) - laneWidthMm;
    var maxY = Math.max.apply(null, ys) + laneWidthMm;
    var padding = 44;
    return { scale: Math.min((canvas.width - padding * 2) / (maxX - minX), (canvas.height - padding * 2) / (maxY - minY)), minX: minX, maxY: maxY, padding: padding };
  }

  function project(transform, node) {
    return { x: transform.padding + (node.x_mm - transform.minX) * transform.scale, y: transform.padding + (transform.maxY - node.y_mm) * transform.scale };
  }

  function status(edge, runtime) {
    if ((runtime.blocked_edge_ids || []).indexOf(edge.edge_id) >= 0) return "blocked";
    if ((runtime.discovered_culvert_edge_ids || []).indexOf(edge.edge_id) >= 0) return "culvert";
    if ((runtime.recon_culvert_edge_ids || []).indexOf(edge.edge_id) >= 0) return "recon";
    if ((runtime.confirmed_clear_edge_ids || []).indexOf(edge.edge_id) >= 0) return "clear";
    return "unknown";
  }

  function drawFeature(context, transform, first, second, kind, solid) {
    var a = project(transform, first), b = project(transform, second);
    var horizontal = Math.abs(a.x - b.x) > Math.abs(a.y - b.y);
    var centerX = (a.x + b.x) / 2, centerY = (a.y + b.y) / 2;
    var lane = laneWidthMm * transform.scale;
    var along = kind === "obstacle" ? 30 : lane * 0.96;
    var across = kind === "obstacle" ? lane * 0.9 : 28;
    context.save();
    context.globalAlpha = solid ? .95 : .7;
    context.fillStyle = kind === "obstacle" ? "#ef4444" : "#2563eb";
    if (horizontal) context.fillRect(centerX - along / 2, centerY - across / 2, along, across);
    else context.fillRect(centerX - across / 2, centerY - along / 2, across, along);
    context.restore();
  }

  function drawVehicle(context, transform, pose) {
    if (!pose) return;
    var position = project(transform, pose), scale = transform.scale;
    var heading = -(pose.yaw_deg || pose.heading_deg || 0) * Math.PI / 180;
    var length = vehicleLengthMm * scale, width = vehicleWidthMm * scale;
    context.save(); context.translate(position.x, position.y); context.rotate(heading);
    context.fillStyle = "#f59e0b"; context.strokeStyle = "#fff"; context.lineWidth = 2;
    context.fillRect(-length / 2, -width / 2, length, width); context.strokeRect(-length / 2, -width / 2, length, width);
    context.fillStyle = "#ef4444"; context.beginPath(); context.moveTo(length / 2 + 8, 0); context.lineTo(length / 2 - 8, -8); context.lineTo(length / 2 - 8, 8); context.closePath(); context.fill(); context.restore();
  }

  window.MapRenderer = function (canvas) {
    var context = canvas.getContext("2d");
    return function (snapshot, mode) {
      var map = snapshot.navigation.map || { nodes: [], edges: [] };
      if (!map.nodes.length) return;
      var runtime = snapshot.navigation.runtime_map || {}, nodes = {};
      map.nodes.forEach(function (node) { nodes[node.node_id] = node; });
      var transform = transformFor(canvas, map.nodes), lane = laneWidthMm * transform.scale;
      var colors = { unknown: "#59636f", clear: "#2e8b57", blocked: "#7f1d1d", culvert: "#174c83", recon: "#1f8a4c" };
      var truth = snapshot.world.truth || {};
      var obstacles = mode === 'truth' ? truth.blocked_edge_ids || [] : runtime.blocked_edge_ids || [];
      var culverts = mode === 'truth' ? truth.culvert_edge_ids || [] : runtime.discovered_culvert_edge_ids || [];
      context.fillStyle = "#101820"; context.fillRect(0, 0, canvas.width, canvas.height);
      map.edges.forEach(function (edge) {
        if (edge.road_kind === "INTERNAL" || !nodes[edge.from_node_id] || !nodes[edge.to_node_id]) return;
        var a = project(transform, nodes[edge.from_node_id]), b = project(transform, nodes[edge.to_node_id]);
        context.strokeStyle = "#29323b"; context.lineWidth = lane + 8; context.lineCap = "butt"; context.beginPath(); context.moveTo(a.x, a.y); context.lineTo(b.x, b.y); context.stroke();
      });
      map.nodes.forEach(function (node) {
        if (node.node_kind === "port" || node.node_kind === "base") return;
        var p = project(transform, node); context.fillStyle = "#29323b"; context.fillRect(p.x - lane / 2 - 4, p.y - lane / 2 - 4, lane + 8, lane + 8);
      });
      map.edges.forEach(function (edge) {
        if (edge.road_kind === "INTERNAL" || !nodes[edge.from_node_id] || !nodes[edge.to_node_id]) return;
        var a = project(transform, nodes[edge.from_node_id]), b = project(transform, nodes[edge.to_node_id]);
        context.strokeStyle = colors[status(edge, runtime)]; context.lineWidth = lane; context.beginPath(); context.moveTo(a.x, a.y); context.lineTo(b.x, b.y); context.stroke();
        context.strokeStyle = "rgba(255,255,255,.2)"; context.lineWidth = 1.5; context.setLineDash([7, 9]); context.beginPath(); context.moveTo(a.x, a.y); context.lineTo(b.x, b.y); context.stroke(); context.setLineDash([]);
        if (obstacles.indexOf(edge.edge_id) >= 0) drawFeature(context, transform, nodes[edge.from_node_id], nodes[edge.to_node_id], "obstacle", mode === 'truth');
        if (culverts.indexOf(edge.edge_id) >= 0) drawFeature(context, transform, nodes[edge.from_node_id], nodes[edge.to_node_id], "culvert", mode === 'truth');
      });
      map.nodes.forEach(function (node) {
        if (node.node_kind === "port") return;
        var p = project(transform, node); context.fillStyle = "#59636f";
        if (node.node_kind === "base") { context.beginPath(); context.arc(p.x, p.y, lane * .35, 0, Math.PI * 2); context.fill(); } else context.fillRect(p.x - lane / 2, p.y - lane / 2, lane, lane);
        context.fillStyle = "#fff"; context.font = "11px sans-serif"; context.textAlign = "center"; context.fillText(node.node_id, p.x, p.y - lane * .65);
      });
      drawVehicle(context, transform, snapshot.world.pose);
    };
  };
}());
