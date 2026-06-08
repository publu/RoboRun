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

  /* ── 3D Scene Viewer (accumulated point cloud + markers) ────── */

  let s3dScene, s3dCamera, s3dRenderer, s3dControls;
  let s3dPoints = null;
  let s3dMarkers = [];
  let s3dCamMarker = null;
  let s3dAnimId = null;
  let s3dRefreshInterval = null;

  function makeTextSprite(text) {
    var canvas = document.createElement("canvas");
    canvas.width = 256;
    canvas.height = 64;
    var ctx = canvas.getContext("2d");
    ctx.fillStyle = "rgba(10, 20, 14, 0.85)";
    ctx.fillRect(0, 0, 256, 64);
    ctx.strokeStyle = "#00d47e";
    ctx.lineWidth = 1;
    ctx.strokeRect(0, 0, 256, 64);
    ctx.font = "bold 22px monospace";
    ctx.fillStyle = "#00ff96";
    ctx.fillText(text, 8, 40);
    var tex = new THREE.CanvasTexture(canvas);
    tex.minFilter = THREE.LinearFilter;
    var mat = new THREE.SpriteMaterial({ map: tex, transparent: true, depthTest: false });
    var sprite = new THREE.Sprite(mat);
    sprite.scale.set(0.6, 0.15, 1);
    return sprite;
  }

  function initScene3D(containerId) {
    var container = document.getElementById(containerId);
    if (!container || typeof THREE === "undefined") return;

    var cw = container.clientWidth || 400;
    var ch = container.clientHeight || 300;

    s3dScene = new THREE.Scene();
    s3dScene.background = new THREE.Color(0x08101a);

    s3dCamera = new THREE.PerspectiveCamera(60, cw / ch, 0.1, 200);
    s3dCamera.position.set(2, 3, 4);
    s3dCamera.lookAt(0, 0, 0);

    s3dRenderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    s3dRenderer.setSize(cw, ch);
    s3dRenderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    s3dRenderer.domElement.style.position = "absolute";
    s3dRenderer.domElement.style.inset = "0";
    container.appendChild(s3dRenderer.domElement);

    if (THREE.OrbitControls) {
      s3dControls = new THREE.OrbitControls(s3dCamera, s3dRenderer.domElement);
      s3dControls.enableDamping = true;
      s3dControls.dampingFactor = 0.08;
      s3dControls.maxDistance = 60;
    }

    var grid = new THREE.GridHelper(30, 30, 0x1a2030, 0x111820);
    s3dScene.add(grid);
    s3dScene.add(new THREE.AxesHelper(1));
    s3dScene.add(new THREE.AmbientLight(0x404060, 0.5));
    var dl = new THREE.DirectionalLight(0xffffff, 0.6);
    dl.position.set(5, 10, 5);
    s3dScene.add(dl);

    // Camera marker
    var cmGeo = new THREE.SphereGeometry(0.06, 12, 12);
    var cmMat = new THREE.MeshPhongMaterial({ color: 0x4090e0, emissive: 0x2060a0 });
    s3dCamMarker = new THREE.Mesh(cmGeo, cmMat);
    s3dScene.add(s3dCamMarker);

    function animate() {
      s3dAnimId = requestAnimationFrame(animate);
      if (s3dControls) s3dControls.update();
      s3dRenderer.render(s3dScene, s3dCamera);
    }
    animate();

    s3dRefreshInterval = setInterval(refreshScene3D, 2000);

    var ro = new ResizeObserver(function () {
      var w = container.clientWidth, h = container.clientHeight;
      if (w === 0 || h === 0) return;
      s3dCamera.aspect = w / h;
      s3dCamera.updateProjectionMatrix();
      s3dRenderer.setSize(w, h);
    });
    ro.observe(container);
  }

  function refreshScene3D() {
    fetch("/api/scene3d")
      .then(function (r) { return r.json(); })
      .then(function (data) {
        if (!data.ok) return;

        // Update stats + placeholder regardless of 3D init state
        var statsEl = document.getElementById("sceneStats");
        if (statsEl) statsEl.textContent = data.count + " pts / " + (data.keyframes || 0) + " kf";
        if (data.count > 0) {
          var ph = document.getElementById("scenePlaceholder");
          if (ph) ph.classList.add("hidden");
        }

        if (!s3dScene) return;

        // Rebuild point cloud
        if (data.points && data.points.length > 0) {
          if (s3dPoints) {
            s3dScene.remove(s3dPoints);
            s3dPoints.geometry.dispose();
            s3dPoints.material.dispose();
          }
          var count = data.points.length;
          var positions = new Float32Array(count * 3);
          var colors = new Float32Array(count * 3);
          for (var i = 0; i < count; i++) {
            var p = data.points[i];
            positions[i * 3] = p[0];
            positions[i * 3 + 1] = p[1];
            positions[i * 3 + 2] = p[2];
            colors[i * 3] = (p[3] || 128) / 255;
            colors[i * 3 + 1] = (p[4] || 128) / 255;
            colors[i * 3 + 2] = (p[5] || 128) / 255;
          }
          var geo = new THREE.BufferGeometry();
          geo.setAttribute("position", new THREE.BufferAttribute(positions, 3));
          geo.setAttribute("color", new THREE.BufferAttribute(colors, 3));
          var mat = new THREE.PointsMaterial({ size: 0.015, vertexColors: true, sizeAttenuation: true });
          s3dPoints = new THREE.Points(geo, mat);
          s3dScene.add(s3dPoints);
        }

        // Clear old markers
        for (var m = 0; m < s3dMarkers.length; m++) {
          s3dScene.remove(s3dMarkers[m].sphere);
          s3dScene.remove(s3dMarkers[m].label);
          s3dMarkers[m].sphere.geometry.dispose();
          s3dMarkers[m].sphere.material.dispose();
          s3dMarkers[m].label.material.map.dispose();
          s3dMarkers[m].label.material.dispose();
        }
        s3dMarkers = [];

        // Add new markers
        if (data.markers) {
          for (var j = 0; j < data.markers.length; j++) {
            var mk = data.markers[j];
            var sg = new THREE.SphereGeometry(0.04, 8, 8);
            var sm = new THREE.MeshPhongMaterial({ color: 0x00d47e, emissive: 0x007a48 });
            var sphere = new THREE.Mesh(sg, sm);
            sphere.position.set(mk.x, mk.y, mk.z);
            s3dScene.add(sphere);

            var label = makeTextSprite(mk.label);
            label.position.set(mk.x, mk.y + 0.12, mk.z);
            s3dScene.add(label);

            s3dMarkers.push({ sphere: sphere, label: label });
          }
        }

        // Camera position
        if (data.camera && s3dCamMarker) {
          s3dCamMarker.position.set(data.camera.x || 0, data.camera.y || 0, data.camera.z || 0);
        }
      })
      .catch(function () {});
  }

  /* ── Cleanup ─────────────────────────────────────────────────── */

  function destroy() {
    if (trajAnimId) cancelAnimationFrame(trajAnimId);
    if (pcAnimId) cancelAnimationFrame(pcAnimId);
    if (s3dAnimId) cancelAnimationFrame(s3dAnimId);
    if (depthInterval) clearInterval(depthInterval);
    if (s3dRefreshInterval) clearInterval(s3dRefreshInterval);
    if (trajRenderer) trajRenderer.dispose();
    if (pcRenderer) pcRenderer.dispose();
    if (s3dRenderer) s3dRenderer.dispose();
  }

  RR.threePanels = {
    initTrajectory: initTrajectory,
    initPointCloud: initPointCloud,
    initDepthHeatmap: initDepthHeatmap,
    refreshPointCloud: refreshPointCloud,
    initScene3D: initScene3D,
    refreshScene3D: refreshScene3D,
    destroy: destroy,
  };
})(window.RoboRun);
