/* WebSocket client for telemetry bus — connects to ws://127.0.0.1:8766 */

window.RoboRun = window.RoboRun || {};

(function (RR) {
  let ws = null;
  let reconnectTimer = null;
  const listeners = [];

  function connect() {
    if (ws && ws.readyState <= 1) return;
    try {
      ws = new WebSocket("ws://127.0.0.1:8766");
    } catch (e) {
      scheduleReconnect();
      return;
    }

    ws.onopen = function () {
      console.log("[telemetry] ws connected");
      if (reconnectTimer) { clearTimeout(reconnectTimer); reconnectTimer = null; }
    };

    ws.onmessage = function (ev) {
      try {
        const msg = JSON.parse(ev.data);
        if (msg.type === "history" && Array.isArray(msg.data)) {
          msg.data.forEach(function (entry) { dispatch(entry); });
        } else {
          dispatch(msg);
        }
      } catch (e) { /* ignore bad frames */ }
    };

    ws.onclose = function () { scheduleReconnect(); };
    ws.onerror = function () { scheduleReconnect(); };
  }

  function scheduleReconnect() {
    if (reconnectTimer) return;
    reconnectTimer = setTimeout(function () {
      reconnectTimer = null;
      connect();
    }, 3000);
  }

  function dispatch(entry) {
    for (let i = 0; i < listeners.length; i++) {
      try { listeners[i](entry); } catch (e) { /* swallow */ }
    }
  }

  RR.telemetryWs = {
    connect: connect,
    onData: function (fn) { listeners.push(fn); },
    isConnected: function () { return ws && ws.readyState === 1; },
  };
})(window.RoboRun);
