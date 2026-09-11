(function () {
    "use strict";

    let lastSnapshot = null;
    let lifecycleRunning = false;
    let snapshotPollTimer = null;
    let snapshotRequestInFlight = false;

    function asNumber(value, fallback) {
        const number = Number(value);
        return Number.isFinite(number) ? number : fallback;
    }

    function mapSnapshot(snapshot) {
        const simulation = snapshot && snapshot.simulation || {};
        const world = snapshot && snapshot.world || simulation.world || {};
        const navigation = snapshot && snapshot.navigation || {};
        const map = navigation.map || {};
        const pose = world.pose || {};
        const rawNodes = Array.isArray(map.nodes) ? map.nodes : Object.values(map.nodes || {});
        const nodes = {};

        rawNodes.forEach((node, index) => {
            const id = String(node.node_id ?? node.id ?? node.name ?? `N${index}`);
            const kind = String(node.node_kind ?? node.type ?? "junction").toLowerCase();
            nodes[id] = {
                x: asNumber(node.x_mm ?? node.x, 0),
                y: -asNumber(node.y_mm ?? node.y, 0),
                type: kind === "start" || id === "START" ? "start" :
                    kind === "mission" || kind === "task" ? "mission" : "junction",
            };
        });

        const edges = (Array.isArray(map.edges) ? map.edges : Object.values(map.edges || {}))
            .map((edge, index) => ({
                edge_id: edge.edge_id ?? edge.id ?? `E${index}`,
                node_a: String(edge.from_node_id ?? edge.node_a ?? edge.from ?? ""),
                node_b: String(edge.to_node_id ?? edge.node_b ?? edge.to ?? ""),
                is_tunnel: String(edge.road_kind ?? edge.type ?? "").toLowerCase().includes("tunnel") ||
                    String(edge.road_kind ?? "").toLowerCase().includes("culvert"),
            }))
            .filter(edge => edge.node_a && edge.node_b);

        const worldX = asNumber(pose.x_mm ?? pose.x, 0);
        const worldY = -asNumber(pose.y_mm ?? pose.y, 0);
        const yaw = asNumber(pose.yaw_deg ?? pose.heading_deg ?? pose.yaw, 90);
        const nodeIds = Object.keys(nodes);
        const nearest = nodeIds.reduce((best, id) => {
            if (!best) return id;
            const currentDistance = (nodes[id].x - worldX) ** 2 + (nodes[id].y - worldY) ** 2;
            const bestDistance = (nodes[best].x - worldX) ** 2 + (nodes[best].y - worldY) ** 2;
            return currentDistance < bestDistance ? id : best;
        }, null);

        const path = navigation.planned_path || navigation.plannedPath || navigation.path ||
            navigation.route?.planned_path || [];
        const visited = navigation.visited_nodes || navigation.visitedNodes || [];
        const trajectory = (world.trajectory || navigation.trajectory || simulation.trajectory || [])
            .map(point => Array.isArray(point) ? [asNumber(point[0], 0), -asNumber(point[1], 0)] : point);
        const currentNode = navigation.current_node || navigation.currentNode || nearest;
        const targetNode = navigation.target_node || navigation.targetNode || null;
        const now = asNumber(simulation.now ?? world.now, Date.now() / 1000);

        return {
            timestamp: now * 1000,
            map_data: { nodes, edges, patrol_path: path },
            position: [worldX, worldY, yaw],
            agent_state: navigation.state || (simulation.running ? "RUNNING" : "IDLE"),
            current_node: currentNode,
            target_node: targetNode,
            planned_path: Array.isArray(path) ? path : [],
            visited_nodes: Array.isArray(visited) ? visited : [],
            trajectory,
            progress: {
                visited: Array.isArray(visited) ? visited.length : 0,
                total: nodeIds.length,
            },
            is_intersection: false,
            events: [],
        };
    }

    function applySnapshot(snapshot) {
        if (!snapshot) return;
        lastSnapshot = snapshot;
        const data = mapSnapshot(snapshot);
        window.__navigation2Snapshot = snapshot;
        window.__MAP_DATA__ = data.map_data;

        if (typeof gNodes !== "undefined") gNodes = data.map_data.nodes;
        if (typeof gEdges !== "undefined") gEdges = data.map_data.edges;
        if (typeof sim !== "undefined") {
            sim.pos = { x: data.position[0], y: data.position[1], yaw: data.position[2] };
            sim.currentNode = data.current_node;
            sim.targetNode = data.target_node;
            sim.path = data.planned_path;
            sim.visited = data.visited_nodes;
            sim.trajectory = data.trajectory;
        }

        const select = document.getElementById("simStartNode");
        if (select && Object.keys(data.map_data.nodes).length && !select.dataset.navigation2Ready) {
            select.innerHTML = Object.keys(data.map_data.nodes).map(id =>
                `<option value="${id}">${id}</option>`).join("");
            select.value = data.current_node || "START";
            select.dataset.navigation2Ready = "1";
        }
        if (typeof updateTelemetry === "function") updateTelemetry(data);
        if (typeof updatePath === "function") updatePath(data);
        if (typeof drawMap === "function") {
            drawMap(data.map_data.nodes, data.map_data.edges, data.position,
                data.visited_nodes, data.current_node, data.target_node,
                data.planned_path, data.trajectory);
        }

        const statusDot = document.getElementById("statusDot");
        if (statusDot) statusDot.classList.add("connected");
        const fpsDisplay = document.getElementById("fpsDisplay");
        if (fpsDisplay) fpsDisplay.textContent = "已连接 | HTTP 快照 10Hz";
    }

    window.onWsMessage = function (event) {
        try {
            applySnapshot(JSON.parse(event.data));
        } catch (error) {
            console.warn("Invalid Navigation 2.0 snapshot", error);
        }
    };

    async function fetchSnapshot() {
        if (snapshotRequestInFlight) return;
        snapshotRequestInFlight = true;
        try {
            const response = await fetch("/api/snapshot");
            if (!response.ok) throw new Error(`HTTP ${response.status}`);
            applySnapshot(await response.json());
        } catch (error) {
            const statusDot = document.getElementById("statusDot");
            if (statusDot) statusDot.classList.remove("connected");
            const fpsDisplay = document.getElementById("fpsDisplay");
            if (fpsDisplay) fpsDisplay.textContent = "快照连接失败";
            console.warn("Navigation 2.0 snapshot unavailable", error);
        } finally {
            snapshotRequestInFlight = false;
        }
    }

    function startSnapshotPolling() {
        if (snapshotPollTimer !== null) return;
        fetchSnapshot();
        snapshotPollTimer = window.setInterval(fetchSnapshot, 100);
    }

    // The Flask development server exposes the canonical HTTP snapshot reliably.
    // Replace the legacy page's WebSocket bootstrap before DOMContentLoaded runs.
    window.connectWebSocket = startSnapshotPolling;
    connectWebSocket = startSnapshotPolling;

    async function sendSimulationCommand(command, payload) {
        try {
            const response = await fetch(`/api/sim/${command}`, {
                method: "POST",
                headers: { "Content-Type": "application/json" },
                body: JSON.stringify(payload || {}),
            });
            const result = await response.json();
            if (!result.ok && typeof logCmd === "function") logCmd(result.error || "命令失败", "error");
            await fetchSnapshot();
        } catch (error) {
            if (typeof logCmd === "function") logCmd(`网络错误: ${error.message}`, "error");
        }
    }

    window.sendApiCmd = function (command, payload) {
        const lifecycle = { stop: "stop", move: "step", turn_imu: "step", intersection_turn: "step" };
        const selected = lifecycle[command];
        if (selected) sendSimulationCommand(selected, payload);
        else if (typeof logCmd === "function") logCmd(`暂不支持命令: ${command}`, "error");
    };

    window.toggleSimulation = function () {
        const command = lifecycleRunning ? "pause" : "start";
        lifecycleRunning = !lifecycleRunning;
        sendSimulationCommand(command);
        const button = document.getElementById("btnSim");
        if (button) button.textContent = lifecycleRunning ? "暂停模拟" : "启动模拟";
    };

    startSnapshotPolling();
})();
