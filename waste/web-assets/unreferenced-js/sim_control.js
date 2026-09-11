/**
 * sim_control.js — 仿真控制面板
 *
 * 提供仿真启动/停止/暂停/单步控制 + 地面真相可视化切换。
 */

// ================================================================
// 仿真控制状态
// ================================================================
let simControl = {
    seed: 42,
    speedMultiplier: 1.0,
    visibleRange: 400,
    autoStart: false,
};

// ================================================================
// API 调用
// ================================================================

async function simApiStart() {
    const seed = parseInt(document.getElementById('simSeed').value) || 42;
    const speed = parseFloat(document.getElementById('simSpeed').value) || 1.0;
    const range = parseInt(document.getElementById('simVisibleRange').value) || 400;

    simControl.seed = seed;
    simControl.speedMultiplier = speed;
    simControl.visibleRange = range;

    try {
        const resp = await fetch('/api/sim/start', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({
                seed: seed,
                speed_multiplier: speed,
                visible_range_mm: range,
            }),
        });
        const data = await resp.json();
        if (data.ok) {
            console.log('[SimControl] 仿真已启动, seed=' + data.seed);
            updateSimButtons('running');
            if (data.scene) {
                simControl.scene = data.scene;
                updateGroundTruthData();
            }
        } else {
            alert('仿真启动失败: ' + (data.error || '未知错误'));
        }
    } catch (e) {
        console.error('[SimControl] 启动失败:', e);
        alert('仿真启动失败: ' + e.message);
    }
}

async function simApiStop() {
    try {
        await fetch('/api/sim/stop', { method: 'POST' });
        console.log('[SimControl] 仿真已停止');
        updateSimButtons('stopped');
        window.__simShowGroundTruth__ = false;
        updateGroundTruthToggle();
    } catch (e) {
        console.error('[SimControl] 停止失败:', e);
    }
}

async function simApiPause() {
    try {
        await fetch('/api/sim/pause', { method: 'POST' });
        console.log('[SimControl] 仿真已暂停');
        updateSimButtons('paused');
    } catch (e) {
        console.error('[SimControl] 暂停失败:', e);
    }
}

async function simApiResume() {
    try {
        await fetch('/api/sim/resume', { method: 'POST' });
        console.log('[SimControl] 仿真已恢复');
        updateSimButtons('running');
    } catch (e) {
        console.error('[SimControl] 恢复失败:', e);
    }
}

async function simApiStep() {
    try {
        const resp = await fetch('/api/sim/step', { method: 'POST' });
        const data = await resp.json();
        if (data.ok) {
            updateSimButtons('paused');
            if (data.snapshot) {
                updateTelemetryFromSimSnapshot(data.snapshot);
            }
            if (!data.has_next) {
                updateSimButtons('stopped');
                console.log('[SimControl] 仿真已完成, state=' + data.state);
            }
        }
    } catch (e) {
        console.error('[SimControl] 单步失败:', e);
    }
}

// ================================================================
// UI 更新
// ================================================================

function updateSimButtons(state) {
    const btnStart = document.getElementById('simBtnStart');
    const btnStop = document.getElementById('simBtnStop');
    const btnPause = document.getElementById('simBtnPause');
    const btnResume = document.getElementById('simBtnResume');
    const btnStep = document.getElementById('simBtnStep');
    const statusEl = document.getElementById('simStatus');

    if (!btnStart) return;  // 仿真面板未加载

    switch (state) {
        case 'running':
            btnStart.disabled = true;
            btnStop.disabled = false;
            btnPause.disabled = false;
            btnResume.disabled = true;
            btnStep.disabled = true;
            if (statusEl) statusEl.textContent = '运行中...';
            if (statusEl) statusEl.className = 'sim-status running';
            break;
        case 'paused':
            btnStart.disabled = true;
            btnStop.disabled = false;
            btnPause.disabled = true;
            btnResume.disabled = false;
            btnStep.disabled = false;
            if (statusEl) statusEl.textContent = '已暂停';
            if (statusEl) statusEl.className = 'sim-status paused';
            break;
        case 'stopped':
        default:
            btnStart.disabled = false;
            btnStop.disabled = true;
            btnPause.disabled = true;
            btnResume.disabled = true;
            btnStep.disabled = false;
            if (statusEl) statusEl.textContent = '未运行';
            if (statusEl) statusEl.className = 'sim-status stopped';
            break;
    }
}

function randomizeSeed() {
    const seedInput = document.getElementById('simSeed');
    if (seedInput) {
        seedInput.value = Math.floor(Math.random() * 100000);
    }
}

// ================================================================
// 地面真相可视化
// ================================================================

function toggleGroundTruth() {
    window.__simShowGroundTruth__ = !window.__simShowGroundTruth__;
    updateGroundTruthToggle();
    if (typeof drawMap === 'function') {
        // 触发重绘
        if (typeof renderMapFromSim === 'function') renderMapFromSim();
        else if (typeof renderMap === 'function') renderMap(gLastSimData || {});
    }
}

function updateGroundTruthToggle() {
    const checkbox = document.getElementById('simShowGT');
    if (checkbox) {
        checkbox.checked = window.__simShowGroundTruth__ || false;
    }
}

function updateGroundTruthData() {
    if (!simControl.scene) return;

    const gt = {
        hidden_obstacles: [],
        hidden_culverts: [],
    };

    // 未发现的障碍物
    const discoveredObs = simControl.scene.discovered_obstacles || [];
    const allObs = simControl.scene.obstacle_edge_ids || [];
    gt.hidden_obstacles = allObs.filter(function(id) {
        return discoveredObs.indexOf(id) === -1;
    });

    // 未发现的涵洞
    const discoveredCul = simControl.scene.discovered_culverts || [];
    const allCul = simControl.scene.culvert_edge_ids || [];
    gt.hidden_culverts = allCul.filter(function(id) {
        return discoveredCul.indexOf(id) === -1;
    });

    window.__simGroundTruthData__ = gt;
}

// 从 WebSocket 数据更新地面真相
function updateGroundTruthFromWs(simData) {
    if (!simData || !simData.scene) return;
    simControl.scene = simData.scene;
    updateGroundTruthData();
    if (window.__simShowGroundTruth__) {
        if (typeof renderMapFromSim === 'function') renderMapFromSim();
        else if (typeof renderMap === 'function') renderMap(gLastSimData || {});
    }
}

// 从仿真快照更新遥测
function updateTelemetryFromSimSnapshot(snapshot) {
    if (typeof updateTelemetry === 'function' && snapshot) {
        updateTelemetry({
            agent_state: snapshot.agent_state,
            position: snapshot.pose ? [snapshot.pose.x_mm, snapshot.pose.y_mm, snapshot.pose.yaw_deg] : null,
            current_node: snapshot.current_node,
            target_node: snapshot.target_node,
            planned_path: snapshot.planned_path,
            visited_nodes: snapshot.visited_nodes,
            progress: snapshot.mission_progress,
        });
    }
    if (typeof renderMapFromSim === 'function') renderMapFromSim();
}

// ================================================================
// 初始化
// ================================================================

function initSimControl() {
    // 初始化全局变量
    window.__simShowGroundTruth__ = false;
    window.__simGroundTruthData__ = null;

    // 绑定按钮事件
    const btnStart = document.getElementById('simBtnStart');
    const btnStop = document.getElementById('simBtnStop');
    const btnPause = document.getElementById('simBtnPause');
    const btnResume = document.getElementById('simBtnResume');
    const btnStep = document.getElementById('simBtnStep');
    const btnRandomSeed = document.getElementById('simBtnRandomSeed');
    const cbShowGT = document.getElementById('simShowGT');

    if (btnStart) btnStart.addEventListener('click', simApiStart);
    if (btnStop) btnStop.addEventListener('click', simApiStop);
    if (btnPause) btnPause.addEventListener('click', simApiPause);
    if (btnResume) btnResume.addEventListener('click', simApiResume);
    if (btnStep) btnStep.addEventListener('click', simApiStep);
    if (btnRandomSeed) btnRandomSeed.addEventListener('click', randomizeSeed);
    if (cbShowGT) cbShowGT.addEventListener('change', toggleGroundTruth);

    updateSimButtons('stopped');
    console.log('[SimControl] 仿真控制面板初始化完成');
}

// 在 DOM 加载后初始化
if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', initSimControl);
} else {
    initSimControl();
}
