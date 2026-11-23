import type { Source, EventMsg, Sample, SceneState, Detection, Pose } from "./Source";
import { apiUrl, wsUrl } from "../runtime";

// Live robot or running sim. Events via SSE, telemetry via the WS on :8766
// (ring-buffered per channel), camera via the MJPEG/frame endpoint. The
// playhead "follows NOW", so sceneAt() ignores t and returns the latest state.
export class LiveSource implements Source {
  kind = "live" as const;
  id = "live";

  private es?: EventSource;
  private ws?: WebSocket;
  private events: EventMsg[] = [];
  private channels = new Map<string, Sample[]>();
  private scene: SceneState = { detections: [] };
  private opened = false;

  // Lazy: don't open SSE+WS until a panel actually consumes this source. The
  // store creates a LiveSource eagerly, but on non-live routes (Analytics,
  // Search, Agent…) nothing reads it, so no connections open.
  private ensure() {
    if (this.opened) return;
    this.opened = true;
    this.es = new EventSource(apiUrl("/api/events/stream"));
    this.es.onmessage = (m) => {
      try {
        const e = JSON.parse(m.data) as EventMsg;
        this.absorb(e);
        this.events.push(e);
        if (this.events.length > 200) this.events.shift();
      } catch {
        /* ping comment lines */
      }
    };
    this.connectWS();
  }

  private connectWS() {
    // The telemetry WS lives on its own port; reach it on the resolved backend
    // host (so a hosted page hits the user's localhost, not the Vercel domain),
    // matching scheme so it isn't blocked as mixed content under https.
    const port = (window as { ROBORUN_WS_PORT?: number }).ROBORUN_WS_PORT ?? 8766;
    const url = wsUrl(port);
    try {
      this.ws = new WebSocket(url);
    } catch {
      return;
    }
    this.ws.onmessage = (m) => {
      try {
        const d = JSON.parse(m.data);
        const rows = d.type === "history" ? d.data : [d];
        for (const r of rows) this.pushSample(r);
      } catch {
        /* ignore */
      }
    };
    this.ws.onclose = () => {
      // best-effort reconnect; the sim/robot may restart the bus
      setTimeout(() => this.connectWS(), 2000);
    };
  }

  private pushSample(r: any) {
    if (!r || !r.channel) return;
    const { channel, robot_id, t, ...rest } = r;
    const buf = this.channels.get(channel) ?? [];
    const nums: Sample = { t: t ?? Date.now() / 1000 };
    for (const [k, v] of Object.entries(rest)) {
      if (typeof v === "number") nums[k] = v;
    }
    buf.push(nums);
    if (buf.length > 600) buf.shift();
    this.channels.set(channel, buf);
  }

  private absorb(e: EventMsg) {
    // keep the latest detections / pose for the scene panel
    const d = e.detail || {};
    if (e.type === "detection" && Array.isArray((d as any).detections)) {
      this.scene.detections = (d as any).detections as Detection[];
    }
    if ((d as any).pose) this.scene.pose = (d as any).pose as Pose;
  }

  bounds() {
    return null; // open-ended: live follows NOW
  }

  snapshotEvents(_t: number) {
    this.ensure();
    return this.events;
  }

  frameURL(_t: number, camera = "auto") {
    this.ensure();
    // cache-bust so the <img> actually refreshes when polled
    return apiUrl(`/api/camera/frame?source=${encodeURIComponent(camera)}&_=${Date.now()}`);
  }

  async series(channel: string) {
    this.ensure();
    return this.channels.get(channel) ?? [];
  }

  sceneAt(_t: number) {
    this.ensure();
    return this.scene;
  }

  dispose() {
    this.es?.close();
    this.ws?.close();
    this.events = [];
  }
}
