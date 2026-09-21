(function () {
  "use strict";
  var mode = "runtime";
  var lastSnapshot = null;
  var renderMap = MapRenderer(document.getElementById("map-canvas"));
  function setText(id, value) { var element = document.getElementById(id); if (element) element.textContent = value == null ? "--" : value; }
  function renderSnapshot(snapshot) {
    lastSnapshot = snapshot;
    var simulation = snapshot.simulation || {}, navigation = snapshot.navigation || {}, world = snapshot.world || {}, pose = world.pose || {}, runtime = navigation.runtime_map || {};
    setText("connection", "online"); setText("clock", new Date().toLocaleTimeString());
    setText("runner-state", simulation.running ? (simulation.paused ? "paused" : "running") : (simulation.stopped ? "stopped" : "idle"));
    setText("sim-time", Number(simulation.now || 0).toFixed(3) + " s"); setText("action-count", simulation.completed_action_count || 0);

    // ---- World pose（SimWorld 真值）----
    setText("pose", pose.x_mm == null ? "--" : Math.round(pose.x_mm) + ", " + Math.round(pose.y_mm) + " / " + Math.round(pose.yaw_deg || pose.heading_deg || 0) + " deg");

    // ---- RobotState pose（计划位姿）----
    var robotPose = simulation.robot_state_pose;
    if (robotPose && robotPose.x_mm != null) {
      setText("robot-pose", Math.round(robotPose.x_mm) + ", " + Math.round(robotPose.y_mm) + " / " + Math.round(robotPose.yaw_deg || 0) + " deg");
    } else {
      setText("robot-pose", "--");
    }

    // ---- 位姿差（Robot - World）----
    if (pose.x_mm != null && robotPose && robotPose.x_mm != null) {
      var dx = robotPose.x_mm - pose.x_mm;
      var dy = robotPose.y_mm - pose.y_mm;
      var dyaw = (robotPose.yaw_deg || 0) - (pose.yaw_deg || pose.heading_deg || 0);
      setText("pose-diff",
        (dx >= 0 ? "+" : "") + dx.toFixed(1) + ", " +
        (dy >= 0 ? "+" : "") + dy.toFixed(1) + " / " +
        (dyaw >= 0 ? "+" : "") + dyaw.toFixed(1) + " deg");
    } else {
      setText("pose-diff", "--");
    }

    // ---- RobotState 逻辑位置 ----
    setText("robot-location", simulation.robot_location_label || "--");

    setText("nav-state", navigation.state); setText("nav-substate", navigation.substate);
    setText("current-action", JSON.stringify(navigation.current_action || null));
    setText("current-request", JSON.stringify(navigation.current_request || null));
    setText("runtime-map-counts", "blocked " + (runtime.blocked_edge_ids || []).length + " | clear " + (runtime.confirmed_clear_edge_ids || []).length + " | culvert " + (runtime.discovered_culvert_edge_ids || []).length);
    setText("diagnostics", (navigation.diagnostics || []).join("\n") || "none");
    var body = document.querySelector("#task-table tbody"); body.innerHTML = "";
    (navigation.tasks || []).forEach(function (task) { var row = document.createElement("tr"); row.innerHTML = "<td>" + task.task_id + "</td><td>" + task.target_id + "</td><td>" + task.lifecycle + "</td>"; body.appendChild(row); });
    var log = document.getElementById("event-log"); log.innerHTML = "";
    (navigation.events || []).slice().reverse().forEach(function (event) { var item = document.createElement("li"); item.innerHTML = "<time>" + Number(event.timestamp || 0).toFixed(3) + "s</time>" + event.message; log.appendChild(item); });
    renderMap(snapshot, mode);
  }
  function command(name) {
    var payload = name === "RESET" ? { seed: Number(document.getElementById("seed").value || 0) } : {}, button = document.querySelector('[data-command="' + name + '"]');
    if (button) button.disabled = true;
    fetch("/api/sim/" + name.toLowerCase(), { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(payload) }).then(function (response) { return response.json().then(function (data) { if (!response.ok) throw Error(data.error || "command failed"); return data; }); }).catch(function (error) { setText("command-error", error.message); }).finally(function () { if (button) button.disabled = false; });
  }
  document.querySelectorAll("[data-command]").forEach(function (button) { button.addEventListener("click", function () { command(button.dataset.command); }); });
  document.getElementById("view-mode").addEventListener("change", function () { mode = this.value; if (lastSnapshot) renderMap(lastSnapshot, mode); });
  SnapshotClient(renderSnapshot, function () { setText("connection", "offline"); }).start();
}());