/* Three.js 3D visualization panels — trajectory, point cloud, depth heatmap */

window.RoboRun = window.RoboRun || {};

(function (RR) {
  /* ── Trajectory Viewer ─────────────────────────────────────── */

  let trajScene, trajCamera, trajRenderer, trajControls;
  let trajLine, trajMarker, trajPoints = [];
  let trajAnimId = null;

  function initTrajectory(containerId) {
    const container = document.getElementById(containerId);
    if (!container || typeof THREE === "undefined") return;

    trajScene = new THREE.Scene();
    trajScene.background = new THREE.Color(0x0a0e14);
    trajScene.fog = new THREE.Fog(0x0a0e14, 20, 60);

    trajCamera = new THREE.PerspectiveCamera(60, container.clientWidth / container.clientHeight, 0.1, 100);
    trajCamera.position.set(3, 5, 3);
    trajCamera.lookAt(0, 0, 0);

    trajRenderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    trajRenderer.setSize(container.clientWidth, container.clientHeight);
    trajRenderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    container.appendChild(trajRenderer.domElement);

    if (THREE.OrbitControls) {
      trajControls = new THREE.OrbitControls(trajCamera, trajRenderer.domElement);
      trajControls.enableDamping = true;
      trajControls.dampingFactor = 0.08;
      trajControls.maxDistance = 40;
    }

    // Grid
    const grid = new THREE.GridHelper(40, 40, 0x1a2030, 0x111820);
    trajScene.add(grid);

    // Axes
    const axes = new THREE.AxesHelper(1.5);
    trajScene.add(axes);

    // Ambient + directional light
    trajScene.add(new THREE.AmbientLight(0x404060, 0.6));
    const dLight = new THREE.DirectionalLight(0xffffff, 0.8);
    dLight.position.set(5, 10, 5);
    trajScene.add(dLight);

    // Trajectory line
    const lineGeo = new THREE.BufferGeometry();
    const lineMat = new THREE.LineBasicMaterial({ color: 0x00d47e, linewidth: 2 });
    trajLine = new THREE.Line(lineGeo, lineMat);
    trajScene.add(trajLine);

    // Robot marker
    const markerGeo = new THREE.SphereGeometry(0.08, 16, 16);
    const markerMat = new THREE.MeshPhongMaterial({ color: 0x40a0e0, emissive: 0x2060a0 });
    trajMarker = new THREE.Mesh(markerGeo, markerMat);
    trajScene.add(trajMarker);

    // Listen for telemetry
    if (RR.telemetryWs) {
      RR.telemetryWs.onData(function (entry) {
        if (entry.channel === "position") {
          trajPoints.push(new THREE.Vector3(entry.x || 0, entry.z || 0, -(entry.y || 0)));
          if (trajPoints.length > 5000) trajPoints.shift();
        }
      });
    }

    function animate() {
      trajAnimId = requestAnimationFrame(animate);

      if (trajPoints.length > 1) {
        const positions = new Float32Array(trajPoints.length * 3);
        for (let i = 0; i < trajPoints.length; i++) {
          positions[i * 3] = trajPoints[i].x;
          positions[i * 3 + 1] = trajPoints[i].y;
          positions[i * 3 + 2] = trajPoints[i].z;
        }
        trajLine.geometry.dispose();
        trajLine.geometry = new THREE.BufferGeometry();
        trajLine.geometry.setAttribute("position", new THREE.BufferAttribute(positions, 3));
      }

      if (trajPoints.length > 0) {
        const last = trajPoints[trajPoints.length - 1];
        trajMarker.position.copy(last);
      }

      if (trajControls) trajControls.update();
      trajRenderer.render(trajScene, trajCamera);
    }
    animate();

    // Resize
    const ro = new ResizeObserver(function () {
      const w = container.clientWidth, h = container.clientHeight;
      trajCamera.aspect = w / h;
      trajCamera.updateProjectionMatrix();
      trajRenderer.setSize(w, h);
    });
    ro.observe(container);
  }

  /* ── Point Cloud Viewer ─────────────────────────────────────── */

  let pcScene, pcCamera, pcRenderer, pcControls;
  let pcPoints = null;
  let pcAnimId = null;

  function initPointCloud(containerId) {
    const container = document.getElementById(containerId);
    if (!container || typeof THREE === "undefined") return;

    pcScene = new THREE.Scene();
    pcScene.background = new THREE.Color(0x08101a);

    pcCamera = new THREE.PerspectiveCamera(60, container.clientWidth / container.clientHeight, 0.1, 100);
    pcCamera.position.set(0, 2, 3);

    pcRenderer = new THREE.WebGLRenderer({ antialias: true });
    pcRenderer.setSize(container.clientWidth, container.clientHeight);
    pcRenderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    container.appendChild(pcRenderer.domElement);

    if (THREE.OrbitControls) {
      pcControls = new THREE.OrbitControls(pcCamera, pcRenderer.domElement);
      pcControls.enableDamping = true;
    }

    pcScene.add(new THREE.AmbientLight(0x404060, 0.4));

    // Axes
    pcScene.add(new THREE.AxesHelper(1));

    function animate() {
      pcAnimId = requestAnimationFrame(animate);
      if (pcControls) pcControls.update();
      pcRenderer.render(pcScene, pcCamera);
    }
    animate();

    // Auto-refresh point cloud
    setInterval(refreshPointCloud, 2000);

    const ro = new ResizeObserver(function () {
      const w = container.clientWidth, h = container.clientHeight;
      pcCamera.aspect = w / h;
      pcCamera.updateProjectionMatrix();
      pcRenderer.setSize(w, h);
    });
    ro.observe(container);
  }

  function refreshPointCloud() {
    fetch("/api/pointcloud")
      .then(function (r) { return r.json(); })
      .then(function (data) {
        if (!data.ok || !data.points || data.points.length === 0) return;
        if (!pcScene) return;

        if (pcPoints) {
          pcScene.remove(pcPoints);
          pcPoints.geometry.dispose();
          pcPoints.material.dispose();
        }

        const count = data.points.length;
        const positions = new Float32Array(count * 3);
        const colors = new Float32Array(count * 3);

        for (let i = 0; i < count; i++) {
          const p = data.points[i];
          positions[i * 3] = p[0];
          positions[i * 3 + 1] = p[1];
          positions[i * 3 + 2] = p[2];
          colors[i * 3] = (p[3] || 128) / 255;
          colors[i * 3 + 1] = (p[4] || 128) / 255;
          colors[i * 3 + 2] = (p[5] || 128) / 255;
        }

        const geo = new THREE.BufferGeometry();
        geo.setAttribute("position", new THREE.BufferAttribute(positions, 3));
        geo.setAttribute("color", new THREE.BufferAttribute(colors, 3));
        const mat = new THREE.PointsMaterial({ size: 0.02, vertexColors: true, sizeAttenuation: true });
        pcPoints = new THREE.Points(geo, mat);
        pcScene.add(pcPoints);
      })
      .catch(function () {});
  }

  /* ── Depth Heatmap ──────────────────────────────────────────── */

  let depthInterval = null;

  function initDepthHeatmap(imgId) {
    const img = document.getElementById(imgId);
    if (!img) return;

    depthInterval = setInterval(function () {
      fetch("/api/depth-frame")
        .then(function (r) { return r.json(); })
        .then(function (data) {
          if (data.ok && data.image) {
            img.src = data.image;
            img.style.display = "block";
            var info = document.getElementById("depthInfo");
            if (info) {
              info.textContent = "Min: " + (data.min_m || 0).toFixed(2) + "m  Max: " + (data.max_m || 0).toFixed(2) + "m";
            }
          }
        })
        .catch(function () {});
    }, 500);
  }

  /* ── Cleanup ─────────────────────────────────────────────────── */

  function destroy() {
    if (trajAnimId) cancelAnimationFrame(trajAnimId);
    if (pcAnimId) cancelAnimationFrame(pcAnimId);
    if (depthInterval) clearInterval(depthInterval);
    if (trajRenderer) trajRenderer.dispose();
    if (pcRenderer) pcRenderer.dispose();
  }

  RR.threePanels = {
    initTrajectory: initTrajectory,
    initPointCloud: initPointCloud,
    initDepthHeatmap: initDepthHeatmap,
    refreshPointCloud: refreshPointCloud,
    destroy: destroy,
  };
})(window.RoboRun);
