import type { Source, EventMsg, Sample, SceneState, Detection } from "./Source";
import { apiUrl } from "../runtime";

// A sealed run, replayed. Events are fetched once and filtered by the playhead;
// frames come per-t from /api/run/frame; series come from /api/run/series
// (named arrays — we map a channel name onto the relevant one). sceneAt(t)
// derives pose + active detections from the events at or before t.
export class RunSource implements Source {
  kind = "run" as const;
  id: string;

  private events: EventMsg[] = [];
  private seriesCache?: Promise<any>;
  private range: { start: number; end: number } | null = null;
  private ready: Promise<void>;

  constructor(runName: string) {
    this.id = runName;
    this.ready = this.load();
  }

  private async load() {
    const r = await fetch(`/api/run/events?run=${encodeURIComponent(this.id)}&limit=100000`);
    const d = await r.json();
    this.events = (d.events || []).filter((e: EventMsg) => typeof e.ts === "number");
    this.events.sort((a, b) => a.ts - b.ts);
    if (this.events.length) {
      this.range = { start: this.events[0].ts, end: this.events[this.events.length - 1].ts };
    }
  }

  whenReady() {
    return this.ready;
  }

  bounds() {
    return this.range;
  }

  // Purely derived from the playhead: every event up to t, newest last. No
  // mutable replay cursor, so scrubbing in either direction stays correct.
  snapshotEvents(t: number) {
    return this.events.filter((e) => e.ts <= t).slice(-200);
  }

  frameURL(t: number, _camera?: string) {
    return apiUrl(`/api/run/frame?id=${encodeURIComponent(this.id)}&t=${t}`);
  }

  private loadSeries() {
    if (!this.seriesCache) {
      this.seriesCache = fetch(`/api/run/series?id=${encodeURIComponent(this.id)}`).then((r) => r.json());
    }
    return this.seriesCache;
  }

  async series(channel: string): Promise<Sample[]> {
    const d = await this.loadSeries();
    // run_series returns named arrays already shaped as {t, ...numbers}
    const arr = (d?.[channel] as Sample[]) || [];
    return arr;
  }

  sceneAt(t: number): SceneState {
    const scene: SceneState = { detections: [] };
    for (const e of this.events) {
      if (e.ts > t) break;
      const det = e.detail || {};
      if (e.type === "detection" && Array.isArray((det as any).detections)) {
        scene.detections = (det as any).detections as Detection[];
      }
      if ((det as any).pose) scene.pose = (det as any).pose;
    }
    return scene;
  }

  dispose() {
    /* nothing to release: no open sockets */
  }
}
