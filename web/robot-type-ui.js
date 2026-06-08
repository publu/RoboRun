/* Robot-type-aware UI adaptation — shows/hides panels based on robot type */

window.RoboRun = window.RoboRun || {};

(function (RR) {
  let currentType = "webcam_only";
  let currentPanels = [];
  let vizInitialized = false;

  function update(robotTypeProfile) {
    if (!robotTypeProfile) return;
    const newType = robotTypeProfile.type || "webcam_only";
    const panels = robotTypeProfile.ui_panels || [];

    if (newType === currentType && panels.length === currentPanels.length) return;
    currentType = newType;
    currentPanels = panels;

    // Robot type badge
    var badge = document.getElementById("robotTypeBadge");
    if (badge) {
      badge.textContent = robotTypeProfile.label || "Unknown";
      badge.style.borderColor = robotTypeProfile.color || "#666";
      badge.style.color = robotTypeProfile.color || "#666";
    }

    // Panels visibility
    toggle("tab-telemetry-wrap", panels.indexOf("telemetry") !== -1);
    toggle("tab-viz-wrap", panels.indexOf("trajectory3d") !== -1 || panels.indexOf("pointcloud") !== -1);
    toggle("depthPanel", panels.indexOf("depth_heatmap") !== -1);
    toggle("droneControlsSection", panels.indexOf("drone_controls") !== -1);
    toggle("robotActionsBody", panels.indexOf("dpad_controls") !== -1 || panels.indexOf("sport_controls") !== -1);

    // Nav tabs
    toggleNav("telemetry", panels.indexOf("telemetry") !== -1);
    toggleNav("viz", panels.indexOf("trajectory3d") !== -1 || panels.indexOf("pointcloud") !== -1);

    // Update telemetry charts for this robot type
    if (RR.telemetryCharts) {
      RR.telemetryCharts.setRobotType(newType);
    }

    // Initialize 3D panels lazily
    if (!vizInitialized && (panels.indexOf("trajectory3d") !== -1 || panels.indexOf("pointcloud") !== -1)) {
      vizInitialized = true;
      setTimeout(function () {
        if (RR.threePanels) {
          if (panels.indexOf("trajectory3d") !== -1) RR.threePanels.initTrajectory("trajectoryCanvas");
          if (panels.indexOf("pointcloud") !== -1) RR.threePanels.initPointCloud("pointCloudCanvas");
        }
      }, 100);
    }

    if (panels.indexOf("depth_heatmap") !== -1 && RR.threePanels) {
      RR.threePanels.initDepthHeatmap("depthImg");
    }
  }

  function toggle(id, show) {
    var el = document.getElementById(id);
    if (el) el.style.display = show ? "" : "none";
  }

  function toggleNav(tab, show) {
    var btn = document.querySelector('.rnav[data-tab="' + tab + '"]');
    if (btn) btn.style.display = show ? "" : "none";
  }

  function getType() { return currentType; }

  RR.robotTypeUI = {
    update: update,
    getType: getType,
  };
})(window.RoboRun);
