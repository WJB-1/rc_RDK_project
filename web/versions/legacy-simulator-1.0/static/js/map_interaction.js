/**
 * map_interaction.js — RoboCup 仿真器地图交互层
 *
 * 依赖：
 *  - map_renderer.js 的 drawMap 会在绘制末尾把命中区域收集到 window.__hitRegions
 *  - drawMap 会根据 window.__hoverBlockName__ 高亮方块端口
 *
 * 职责：
 *  - hover 命中检测（方块 / 直道 / 端口）
 *  - tooltip 展示
 *  - 点击选中实体，展示详情面板
 *  - 维护 window.__hoverBlockName__ / __hoverEdgeId__ / __selectedEntity__ 全局状态
 *
 * 本文件自包含，无第三方依赖，DOM 约定全部防御式处理，缺失不报错。
 */

(function () {
  'use strict';

  // 模块级变量：画布引用（initMapInteraction 时填充）
  var canvas = null;
  var tooltipEl = null;
  var detailPanelEl = null;
  var detailBodyEl = null;
  var btnCloseDetailEl = null;

  // 记录上一次命中，用于判断是否触发重绘（避免 mousemove 疯狂重绘）
  var lastHoverBlockName = null;
  var lastHoverEdgeId = null;
  var lastHoverObsId = null;
  var lastHoverCulId = null;

  /* ------------------------------------------------------------------ */
  /* 类型翻译                                                            */
  /* ------------------------------------------------------------------ */

  function translateType(type) {
    switch (type) {
      case 'corner':
        return '拐角(2岔)';
      case 'junction-T':
        return '丁字路口(3岔)';
      case 'junction-cross':
        return '十字路口(4岔)';
      case 'base':
        return '出发点';
      default:
        return type || '未知';
    }
  }

  /* ------------------------------------------------------------------ */
  /* 初始化                                                              */
  /* ------------------------------------------------------------------ */

  function initMapInteraction() {
    canvas = document.getElementById('mapCanvas');
    if (!canvas) {
      return;
    }

    // 缓存 DOM 引用（允许为 null，后续操作防御处理）
    tooltipEl = document.getElementById('hoverTooltip');
    detailPanelEl = document.getElementById('detailPanel');
    detailBodyEl = document.getElementById('detailPanelBody');
    btnCloseDetailEl = document.getElementById('btnCloseDetail');

    canvas.addEventListener('mousemove', handleMouseMove);
    canvas.addEventListener('click', handleClick);
    canvas.addEventListener('mouseleave', handleMouseLeave);

    // 关闭按钮：清空选中并隐藏详情
    if (btnCloseDetailEl) {
      btnCloseDetailEl.addEventListener('click', function () {
        window.__selectedEntity__ = null;
        hideDetailPanel();
        triggerRedraw();
      });
    }
  }

  /* ------------------------------------------------------------------ */
  /* 坐标换算                                                            */
  /* ------------------------------------------------------------------ */

  // 把鼠标事件坐标换算成 canvas 的逻辑坐标（rect 坐标系，未 × dpr）
  function _mouseToLogical(e) {
    var rect = canvas.getBoundingClientRect();
    return { x: e.clientX - rect.left, y: e.clientY - rect.top };
  }

  /* ------------------------------------------------------------------ */
  /* 命中检测                                                            */
  /* ------------------------------------------------------------------ */

  // 点到线段距离
  function distToSeg(px, py, ax, ay, bx, by) {
    var dx = bx - ax;
    var dy = by - ay;
    var len2 = dx * dx + dy * dy;
    if (len2 === 0) {
      return Math.hypot(px - ax, py - ay);
    }
    var t = ((px - ax) * dx + (py - ay) * dy) / len2;
    t = Math.max(0, Math.min(1, t));
    return Math.hypot(px - (ax + t * dx), py - (ay + t * dy));
  }

  // 命中检测：障碍/涵洞 > 方块中心 > 直道；不命中返回 null
  function hitTest(lx, ly) {
    var regions = window.__hitRegions;
    if (!regions) {
      return null;
    }

    // 涵洞（先查：面积大，盖在直道上）
    var culverts = regions.culverts;
    var i, c;
    if (culverts) {
      for (i = 0; i < culverts.length; i++) {
        c = culverts[i];
        if (Math.abs(lx - c.cx) <= c.halfW && Math.abs(ly - c.cy) <= c.halfH) {
          return {
            type: 'culvert',
            edge_id: c.edge_id, offset_mm: c.offset_mm,
            cx: c.cx, cy: c.cy, halfW: c.halfW, halfH: c.halfH,
            isHorizontal: c.isHorizontal,
            discovered: c.discovered, recon: c.recon,
          };
        }
      }
    }

    // 障碍物
    var obstacles = regions.obstacles;
    if (obstacles) {
      for (i = 0; i < obstacles.length; i++) {
        var o = obstacles[i];
        if (Math.abs(lx - o.cx) <= o.halfW && Math.abs(ly - o.cy) <= o.halfH) {
          return {
            type: 'obstacle',
            edge_id: o.edge_id, offset_mm: o.offset_mm,
            cx: o.cx, cy: o.cy, halfW: o.halfW, halfH: o.halfH,
            isHorizontal: o.isHorizontal,
            discovered: o.discovered,
          };
        }
      }
    }

    // 方块：轴对齐矩形判定
    var blocks = regions.blocks;
    var i, b;
    if (blocks) {
      for (i = 0; i < blocks.length; i++) {
        b = blocks[i];
        if (Math.abs(lx - b.px) <= b.half && Math.abs(ly - b.py) <= b.half) {
          return { type: 'block', name: b.name, node: b.node };
        }
      }
    }

    // 直道：点到线段距离判定
    var edges = regions.edges;
    var e;
    if (edges) {
      for (i = 0; i < edges.length; i++) {
        e = edges[i];
        if (distToSeg(lx, ly, e.ax, e.ay, e.bx, e.by) <= e.half) {
          return { type: 'edge', edge_id: e.edge_id, edge: e.edge };
        }
      }
    }

    return null;
  }

  /* ------------------------------------------------------------------ */
  /* 事件处理                                                            */
  /* ------------------------------------------------------------------ */

  function handleMouseMove(e) {
    var pos = _mouseToLogical(e);
    var hit = hitTest(pos.x, pos.y);

    var newBlockName = null;
    var newEdgeId = null;
    var newObsId = null;
    var newCulId = null;

    if (hit && hit.type === 'block') {
      newBlockName = hit.name;
    } else if (hit && hit.type === 'edge') {
      newEdgeId = hit.edge_id;
    } else if (hit && hit.type === 'obstacle') {
      newObsId = hit.edge_id;
    } else if (hit && hit.type === 'culvert') {
      newCulId = hit.edge_id;
    }

    // 更新全局状态
    window.__hoverBlockName__ = newBlockName;
    window.__hoverEdgeId__ = newEdgeId;

    // cursor：命中实体时变手型，移开恢复
    if (canvas) {
      canvas.style.cursor = hit ? 'pointer' : '';
    }

    // 命中状态变化才触发重绘
    if (newBlockName !== lastHoverBlockName || newEdgeId !== lastHoverEdgeId ||
        newObsId !== lastHoverObsId || newCulId !== lastHoverCulId) {
      lastHoverBlockName = newBlockName;
      lastHoverEdgeId = newEdgeId;
      lastHoverObsId = newObsId;
      lastHoverCulId = newCulId;
      triggerRedraw();
    }

    // tooltip
    if (hit) {
      showTooltip(buildTooltip(hit), pos.x, pos.y);
    } else {
      hideTooltip();
    }
  }

  function handleClick(e) {
    var pos = _mouseToLogical(e);
    var hit = hitTest(pos.x, pos.y);

    if (hit) {
      window.__selectedEntity__ = hit;
      showDetailPanel(buildDetail(hit));
    } else {
      window.__selectedEntity__ = null;
      hideDetailPanel();
    }
  }

  function handleMouseLeave() {
    // 清空 hover 状态
    window.__hoverBlockName__ = null;
    window.__hoverEdgeId__ = null;
    lastHoverBlockName = null;
    lastHoverEdgeId = null;
    lastHoverObsId = null;
    lastHoverCulId = null;
    if (canvas) {
      canvas.style.cursor = '';
    }
    hideTooltip();
    triggerRedraw();
  }

  // 触发重绘：调用全局 drawMap（若存在且为函数）
  function triggerRedraw() {
    if (typeof window.drawMap === 'function') {
      window.drawMap();
    }
  }

  /* ------------------------------------------------------------------ */
  /* tooltip 构建与展示                                                  */
  /* ------------------------------------------------------------------ */

  function buildTooltip(hit) {
    var html;
    if (hit.type === 'block') {
      var node = hit.node || {};
      html = '<b>' + hit.name + '</b>';
      if (node.has_rfid) {
        html += node.is_visited ? ' · ✓已打卡' : ' · 未打卡';
      }
      return html;
    }

    if (hit.type === 'edge') {
      var edge = hit.edge || {};
      var status = '';
      if (edge.is_tunnel) status += ' 🔵';
      if (edge.is_reconned) status += ' ✅';
      else if (edge.has_culvert) status += ' 🔷';
      if (edge.is_blocked) status += ' ✗';
      return edge.node_a + ' → ' + edge.node_b + ' · ' + edge.distance_mm + 'mm' + status;
    }

    if (hit.type === 'obstacle') {
      return hit.discovered ? '障碍物 · 已发现' : '障碍物 · 未发现';
    }

    if (hit.type === 'culvert') {
      var culStatus = '未发现';
      if (hit.discovered) {
        culStatus = hit.recon ? '已发现·已探索' : '已发现·未探索';
      }
      return '涵洞 · ' + culStatus;
    }

    return '';
  }

  function showTooltip(html, x, y) {
    if (!tooltipEl) {
      return;
    }
    tooltipEl.innerHTML = html;
    tooltipEl.style.display = 'block';
    // 相对画布容器定位：直接用逻辑坐标（若容器即视口原点则正确），带 12px 偏移
    tooltipEl.style.left = (x + 12) + 'px';
    tooltipEl.style.top = (y + 12) + 'px';
  }

  function hideTooltip() {
    if (!tooltipEl) {
      return;
    }
    tooltipEl.style.display = 'none';
  }

  /* ------------------------------------------------------------------ */
  /* 详情面板                                                            */
  /* ------------------------------------------------------------------ */

  function buildDetail(hit) {
    if (hit.type === 'block') {
      return buildBlockDetail(hit);
    }
    if (hit.type === 'edge') {
      return buildEdgeDetail(hit);
    }
    if (hit.type === 'obstacle') {
      return buildObstacleDetail(hit);
    }
    if (hit.type === 'culvert') {
      return buildCulvertDetail(hit);
    }
    return '';
  }

  function buildObstacleDetail(hit) {
    var discovered = hit.discovered ? '是' : '否';
    return '<table class="detail-table">' +
      '<tr><th>障碍物</th><td>#' + esc(String(hit.edge_id)) + '</td></tr>' +
      '<tr><th>所在边</th><td>' + esc(String(hit.edge_id)) + '</td></tr>' +
      '<tr><th>偏移</th><td>' + esc(String(hit.offset_mm)) + ' mm</td></tr>' +
      '<tr><th>是否被发现</th><td>' + esc(discovered) + '</td></tr>' +
      '</table>';
  }

  function buildCulvertDetail(hit) {
    var discovered = hit.discovered ? '是' : '否';
    var recon = hit.recon ? '是' : '否';
    return '<table class="detail-table">' +
      '<tr><th>涵洞</th><td>#' + esc(String(hit.edge_id)) + '</td></tr>' +
      '<tr><th>所在边</th><td>' + esc(String(hit.edge_id)) + '</td></tr>' +
      '<tr><th>偏移</th><td>' + esc(String(hit.offset_mm)) + ' mm</td></tr>' +
      '<tr><th>是否被发现</th><td>' + esc(discovered) + '</td></tr>' +
      '<tr><th>是否被探索</th><td>' + esc(recon) + '</td></tr>' +
      '</table>';
  }

  function buildBlockDetail(hit) {
    var node = hit.node || {};
    var x = node.x != null ? node.x : '-';
    var y = node.y != null ? node.y : '-';
    var rfid = node.has_rfid ? '是' : '否';
    var visited = node.has_rfid ? (node.is_visited ? '✓已打卡' : '未打卡') : '-';

    return '<table class="detail-table">' +
      '<tr><th>名称</th><td>' + esc(hit.name) + '</td></tr>' +
      '<tr><th>类型</th><td>' + esc(translateType(node.type)) + '</td></tr>' +
      '<tr><th>坐标</th><td>' + esc(String(x)) + ', ' + esc(String(y)) + '</td></tr>' +
      '<tr><th>RFID</th><td>' + esc(rfid) + '</td></tr>' +
      '<tr><th>打卡</th><td>' + esc(visited) + '</td></tr>' +
      '</table>';
  }

  function buildEdgeDetail(hit) {
    var edge = hit.edge || {};
    var tunnel = edge.is_tunnel ? '是' : '否';
    // 三态：已探索 > 已发现 > 无
    var culvert = edge.is_reconned ? '有（已探索）' : (edge.has_culvert ? '有（已发现）' : '无');
    var blocked = edge.is_blocked ? '✗ 是' : '否';
    var passed = edge.visit_count != null ? edge.visit_count : '-';
    var speed = edge.speed_limit_ms != null ? (edge.speed_limit_ms * 1000).toFixed(0) + ' mm/s' : '-';

    return '<table class="detail-table">' +
      '<tr><th>直道</th><td>' + esc(String(hit.edge_id)) + '</td></tr>' +
      '<tr><th>两端</th><td>' + esc(edge.node_a || '') + ' → ' + esc(edge.node_b || '') + '</td></tr>' +
      '<tr><th>长度</th><td>' + esc(String(edge.distance_mm)) + ' mm</td></tr>' +
      '<tr><th>隧道</th><td>' + esc(tunnel) + '</td></tr>' +
      '<tr><th>涵洞</th><td>' + esc(culvert) + '</td></tr>' +
      '<tr><th>封锁</th><td>' + esc(blocked) + '</td></tr>' +
      '<tr><th>经过次数</th><td>' + esc(String(passed)) + '</td></tr>' +
      '<tr><th>限速</th><td>' + esc(speed) + '</td></tr>' +
      '</table>';
  }

  function showDetailPanel(html) {
    if (!detailPanelEl) {
      return;
    }
    if (detailBodyEl) {
      detailBodyEl.innerHTML = html;
    }
    detailPanelEl.style.display = 'block';
  }

  function hideDetailPanel() {
    if (!detailPanelEl) {
      return;
    }
    detailPanelEl.style.display = 'none';
  }

  // HTML 转义，防止节点名等字段注入
  function esc(s) {
    if (s == null) return '';
    return String(s)
      .replace(/&/g, '&amp;')
      .replace(/</g, '&lt;')
      .replace(/>/g, '&gt;')
      .replace(/"/g, '&quot;')
      .replace(/'/g, '&#39;');
  }

  /* ------------------------------------------------------------------ */
  /* 导出                                                                */
  /* ------------------------------------------------------------------ */

  // 暴露给外部（HTML 内联脚本调用 initMapInteraction）
  window.initMapInteraction = initMapInteraction;
  window.mapHitTest = hitTest;
})();
