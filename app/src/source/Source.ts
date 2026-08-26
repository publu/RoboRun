// The spine of Studio: panels bind to a Source and never know whether they
// are showing a live robot/sim or a sealed run. Two implementations:
// LiveSource (follows NOW) and RunSource (scrubs a recorded run).

export type EventMsg = {
  id?: string;
  type: string;
  source: string;
  title: string;
  detail?: Record<string, unknown>;
  ts: number;
  prev?: string;
};

export type Detection = {
  label: string;
  confidence?: number;
  bbox?: [number, number, number, number];
};

export type Pose = { x: number; y: number; z: number };

export type SceneState = {
  pose?: Pose;
  detections: Detection[];
};

export type Sample = { t: number; [k: string]: number };

export interface Source {
  kind: "live" | "run";
  id: string; // 'live' | run name

  /** [start,end] in unix seconds, or null for an open-ended live source. */
  bounds(): { start: number; end: number } | null;

  /** Events visible at the playhead. Live: rolling buffer (t ignored). Run:
   *  every event with ts <= t. Purely derived from t, so scrubbing back and
   *  forth never duplicates or loses entries. */
  snapshotEvents(t: number): EventMsg[];

  /** URL for a camera frame at time t (live: latest; run: nearest to t). */
  frameURL(t: number, camera?: string): string;

  /** Telemetry series for a channel (live: ring buffer; run: full series). */
  series(channel: string, robot?: string): Promise<Sample[]>;

  /** Poses + detections for the 3D/detection panels at time t. */
  sceneAt(t: number): SceneState;

  /** Release any open connections (EventSource, WS). */
  dispose(): void;
}
