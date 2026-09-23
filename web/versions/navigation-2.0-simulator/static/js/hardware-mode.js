(function () {
  "use strict";

  function selectMode(mode) {
    var radio = document.querySelector('input[name="debug-mode"][value="' + mode + '"]');
    if (radio) radio.checked = true;
    var step = document.querySelector('[data-command="STEP"]');
    if (step) step.disabled = mode === "auto";
  }

  function localizeState(elementId) {
    var labels = {
      idle: "空闲",
      planning: "规划中",
      executing: "运动执行中",
      observing: "环境观察中",
      task_processing: "任务处理中",
      recovering: "恢复处理中",
      completed: "任务完成",
      failed: "故障"
    };
    var element = document.getElementById(elementId);
    if (!element) return;
    new MutationObserver(function () {
      var localized = labels[element.textContent];
      if (localized) element.textContent = localized;
    }).observe(element, {childList: true});
  }

  function sendMode(mode) {
    fetch("/api/sim/set_mode", {
      method: "POST",
      headers: {"Content-Type": "application/json"},
      body: JSON.stringify({mode: mode})
    }).then(function (response) {
      return response.json().then(function (data) {
        if (!response.ok) throw Error(data.error || "模式切换失败");
        return data;
      });
    }).then(function () {
      selectMode(mode);
      document.getElementById("command-error").textContent = "";
    }).catch(function (error) {
      document.getElementById("command-error").textContent = error.message;
    });
  }

  document.querySelectorAll('input[name="debug-mode"]').forEach(function (radio) {
    radio.addEventListener("change", function () {
      if (radio.checked) sendMode(radio.value);
    });
  });
  localizeState("nav-state");
  localizeState("nav-substate");

  setInterval(function () {
    fetch("/api/snapshot", {cache: "no-store"}).then(function (response) {
      return response.json();
    }).then(function (snapshot) {
      selectMode((snapshot.simulation || {}).debug_mode || "manual");
    }).catch(function () {});
  }, 1000);
}());
