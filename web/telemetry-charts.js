/* Telemetry charts — Chart.js real-time dashboards driven by WebSocket data */

window.RoboRun = window.RoboRun || {};

(function (RR) {
  const MAX_POINTS = 120;
  const charts = {};
  const buffers = {};
  let robotType = "webcam_only";
  let initialized = false;

  const CHART_DEFS = {
    battery:  { label: "Battery %",  color: "#00d47e", min: 0, max: 100, field: "percent",  types: ["drone","quadruped","humanoid"] },
    altitude: { label: "Altitude m", color: "#40a0e0", min: 0, max: 10,  field: "z",        types: ["drone","quadruped","humanoid"] },
    speed:    { label: "Speed m/s",  color: "#e07040", min: 0, max: 5,   field: "_speed",    types: ["drone","quadruped","humanoid"] },
    roll:     { label: "Roll deg",   color: "#d4a030", min: -90, max: 90, field: "roll",     types: ["drone","quadruped","humanoid"] },
    pitch:    { label: "Pitch deg",  color: "#a060f0", min: -90, max: 90, field: "pitch",    types: ["drone","quadruped","humanoid"] },
    yaw:      { label: "Yaw deg",    color: "#e04040", min: -180, max: 180, field: "yaw",    types: ["drone","quadruped","humanoid"] },
    fps:      { label: "FPS",        color: "#40a0e0", min: 0, max: 60,  field: "fps",       types: ["webcam_only"] },
  };

  function init() {
    if (initialized) return;
    if (typeof Chart === "undefined") return;
    initialized = true;

    Chart.defaults.color = "#a0a0a0";
    Chart.defaults.borderColor = "rgba(255,255,255,0.06)";
    Chart.defaults.font.family = "'JetBrains Mono', monospace";
    Chart.defaults.font.size = 10;

    Object.keys(CHART_DEFS).forEach(function (key) {
      buffers[key] = { labels: [], data: [] };
    });

    RR.telemetryWs.onData(onTelemetry);
    setInterval(updateCharts, 250);
  }

  function setRobotType(type) {
    robotType = type || "webcam_only";
    rebuildCharts();
  }

  function rebuildCharts() {
    const grid = document.getElementById("telemetryGrid");
    if (!grid) return;
    grid.innerHTML = "";

    Object.keys(charts).forEach(function (k) {
      try { charts[k].destroy(); } catch (e) {}
      delete charts[k];
    });

    Object.keys(CHART_DEFS).forEach(function (key) {
      const def = CHART_DEFS[key];
      if (def.types.indexOf(robotType) === -1) return;

      const card = document.createElement("div");
      card.className = "tel-chart-card";
      card.innerHTML = '<div class="tel-chart-head"><span class="tel-chart-label">' +
        def.label + '</span><span class="tel-chart-val" id="telVal_' + key + '">--</span></div>' +
        '<canvas id="telChart_' + key + '"></canvas>';
      grid.appendChild(card);

      const ctx = document.getElementById("telChart_" + key);
      if (!ctx) return;
      charts[key] = new Chart(ctx, {
        type: "line",
        data: {
          labels: [],
          datasets: [{
            data: [],
            borderColor: def.color,
            backgroundColor: def.color + "18",
            borderWidth: 1.5,
            pointRadius: 0,
            fill: true,
            tension: 0.3,
          }]
        },
        options: {
          responsive: true,
          maintainAspectRatio: false,
          animation: false,
          plugins: { legend: { display: false } },
          scales: {
            x: { display: false },
            y: { min: def.min, max: def.max, ticks: { maxTicksLimit: 4 } },
          },
        },
      });
    });
  }

  function onTelemetry(entry) {
    const ch = entry.channel;
    if (!ch) return;

    if (ch === "battery" && entry.percent !== undefined) {
      pushData("battery", entry.percent);
    }
    if (ch === "position" && entry.z !== undefined) {
      pushData("altitude", entry.z);
    }
    if (ch === "velocity") {
      const spd = Math.sqrt(
        (entry.x || 0) * (entry.x || 0) +
        (entry.y || 0) * (entry.y || 0) +
        (entry.z || 0) * (entry.z || 0)
      );
      pushData("speed", Math.round(spd * 100) / 100);
    }
    if (ch === "orientation") {
      if (entry.roll !== undefined) pushData("roll", toDeg(entry.roll));
      if (entry.pitch !== undefined) pushData("pitch", toDeg(entry.pitch));
      if (entry.yaw !== undefined) pushData("yaw", toDeg(entry.yaw));
    }
  }

  function toDeg(rad) { return Math.round(rad * 180 / Math.PI * 10) / 10; }

  function pushData(key, value) {
    const buf = buffers[key];
    if (!buf) return;
    const now = new Date();
    const label = now.getMinutes() + ":" + String(now.getSeconds()).padStart(2, "0");
    buf.labels.push(label);
    buf.data.push(value);
    if (buf.labels.length > MAX_POINTS) {
      buf.labels.shift();
      buf.data.shift();
    }
  }

  function updateCharts() {
    Object.keys(charts).forEach(function (key) {
      const chart = charts[key];
      const buf = buffers[key];
      if (!chart || !buf) return;
      chart.data.labels = buf.labels.slice();
      chart.data.datasets[0].data = buf.data.slice();
      chart.update("none");

      const valEl = document.getElementById("telVal_" + key);
      if (valEl && buf.data.length > 0) {
        valEl.textContent = buf.data[buf.data.length - 1].toFixed(1);
      }
    });
  }

  RR.telemetryCharts = {
    init: init,
    setRobotType: setRobotType,
    rebuildCharts: rebuildCharts,
  };
})(window.RoboRun);
