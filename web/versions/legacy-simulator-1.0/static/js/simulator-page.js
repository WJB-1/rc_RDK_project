(() => {
    'use strict';

    const state = {
        nodes: {},
        edges: [],
        scene: null,
        running: false,
        paused: false,
        seed: 42,
        position: null,
        currentNode: '',
        targetNode: '',
        visitedNodes: [],
        plannedPath: [],
        trajectory: [],
        progress: null,
    };

    const canvas = document.getElementById('mapCanvas');
    const getElement = (id) => document.getElementById(id);

    function draw() {
        if (!canvas || typeof window.drawMap !== 'function') return;
        window.drawMap(
            state.nodes,
            state.edges,
            state.position,
            state.visitedNodes,
            state.currentNode,
            state.targetNode,
            state.plannedPath,
            state.trajectory,
        );
    }

    function setConnection(connected) {
        const dot = getElement('wsDot');
        const text = getElement('statusText');
        if (dot) dot.className = `ws-indicator ${connected ? 'ws-connected' : 'ws-disconnected'}`;
        if (text) text.textContent = connected ? '已连接' : '连接断开';
    }

    function updateControls() {
        const hasScene = Boolean(state.scene);
        getElement('btnGenScene').disabled = state.running;
        getElement('btnStartSim').disabled = state.running || !hasScene;
        getElement('btnStopSim').disabled = !state.running;
        getElement('btnPauseSim').disabled = !state.running || state.paused;
        getElement('btnResumeSim').disabled = !state.running || !state.paused;
        getElement('btnStepSim').disabled = state.running && !state.paused;

        const badge = getElement('simBadge');
        if (!badge) return;
        if (!hasScene) {
            badge.textContent = '未就绪';
            badge.className = 'status-badge status-stopped';
        } else if (state.paused) {
            badge.textContent = '已暂停';
            badge.className = 'status-badge status-paused';
        } else if (state.running) {
            badge.textContent = '运行中';
            badge.className = 'status-badge status-running';
        } else {
            badge.textContent = '场景就绪';
            badge.className = 'status-badge status-stopped';
        }
    }

    function updateTelemetry() {
        if (state.position) {
            const [x, y, yaw] = state.position;
            getElement('pos').textContent = `${Number(x).toFixed(0)}, ${Number(y).toFixed(0)}`;
            getElement('yawVal').textContent = `${Number(yaw || 0).toFixed(1)}°`;
        }
        if (state.currentNode) getElement('curNode').textContent = state.currentNode;
        if (state.targetNode) getElement('tgtNode').textContent = state.targetNode;
        if (state.progress) {
            const visited = state.progress.visited || 0;
            const total = state.progress.total || 0;
            getElement('mission').textContent = `${visited}/${total}`;
            getElement('progressBar').style.width = total ? `${visited / total * 100}%` : '0%';
        }
    }

    function updateStats() {
        if (!state.scene) return;
        const obstacles = state.scene.obstacle_edge_ids || [];
        const culverts = state.scene.culvert_edge_ids || [];
        getElement('statObstacles').textContent = `0/${obstacles.length}`;
        getElement('statCulverts').textContent = `0/${culverts.length}`;
        getElement('statReconCulverts').textContent = `0/${culverts.length}`;
        getElement('statSeed').textContent = state.seed;
    }

    function applySnapshot(data) {
        const mapData = data.map_data || {};
        if (mapData.nodes && Object.keys(mapData.nodes).length) state.nodes = mapData.nodes;
        if (mapData.edges) state.edges = mapData.edges;
        if (data.position) state.position = data.position;
        if (data.current_node !== undefined) state.currentNode = data.current_node;
        if (data.target_node !== undefined) state.targetNode = data.target_node;
        if (data.visited_nodes) state.visitedNodes = data.visited_nodes;
        if (data.planned_path) state.plannedPath = data.planned_path;
        if (data.trajectory) state.trajectory = data.trajectory;
        if (data.progress) state.progress = data.progress;
        if (data.sim) {
            state.scene = data.sim.scene || state.scene;
            state.running = Boolean(data.sim.running);
            state.seed = data.sim.seed ?? state.seed;
        }
        updateTelemetry();
        updateStats();
        updateControls();
        draw();
    }

    async function request(url, options) {
        const response = await fetch(url, options);
        const data = await response.json();
        if (!response.ok || data.ok === false) throw new Error(data.error || '请求失败');
        return data;
    }

    async function generateScene() {
        state.seed = Number.parseInt(getElement('simSeed').value, 10) || 42;
        const data = await request('/api/sim/scene/generate', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ seed: state.seed }),
        });
        state.scene = data.scene;
        updateStats();
        updateControls();
        draw();
    }

    async function startSimulation() {
        const speed = Number.parseFloat(getElement('speedSlider').value) || 1;
        const data = await request('/api/sim/start', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ seed: state.seed, speed_multiplier: speed }),
        });
        state.scene = data.scene || state.scene;
        state.running = true;
        state.paused = false;
        updateControls();
    }

    async function control(url, paused) {
        await request(url, { method: 'POST' });
        state.paused = paused;
        if (url.endsWith('/stop')) state.running = false;
        updateControls();
    }

    function bindControls() {
        getElement('btnRandomSeed').addEventListener('click', () => {
            getElement('simSeed').value = Math.floor(Math.random() * 100000);
        });
        getElement('btnGenScene').addEventListener('click', () => generateScene().catch((error) => alert(error.message)));
        getElement('btnStartSim').addEventListener('click', () => startSimulation().catch((error) => alert(error.message)));
        getElement('btnStopSim').addEventListener('click', () => control('/api/sim/stop', false).catch((error) => alert(error.message)));
        getElement('btnPauseSim').addEventListener('click', () => control('/api/sim/pause', true).catch((error) => alert(error.message)));
        getElement('btnResumeSim').addEventListener('click', () => control('/api/sim/resume', false).catch((error) => alert(error.message)));
        getElement('speedSlider').addEventListener('input', (event) => {
            getElement('speedVal').textContent = `${event.target.value}x`;
        });
    }

    function connectWebSocket() {
        const socket = new WebSocket(`ws://${location.host}/ws`);
        socket.addEventListener('open', () => setConnection(true));
        socket.addEventListener('message', (event) => {
            try { applySnapshot(JSON.parse(event.data)); } catch (_) {}
        });
        socket.addEventListener('close', () => {
            setConnection(false);
            window.setTimeout(connectWebSocket, 1000);
        });
    }

    async function initialize() {
        bindControls();
        try { applySnapshot(await request('/snapshot')); } catch (_) { draw(); }
        const observer = new ResizeObserver(draw);
        observer.observe(canvas.parentElement);
        connectWebSocket();
        updateControls();
    }

    window.addEventListener('DOMContentLoaded', initialize);
})();
