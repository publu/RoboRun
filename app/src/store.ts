import { create } from "zustand";
import type { Source } from "./source/Source";
import { LiveSource } from "./source/LiveSource";
import { RunSource } from "./source/RunSource";

type StudioState = {
  source: Source;
  t: number; // playhead, unix seconds
  playing: boolean;
  follow: boolean; // live: pin t to NOW
  scopeKey: number; // bumped on project switch to refetch scoped data (no reload)
  bumpScope: () => void;
  tourStep: number; // 0 = off; 1..N = guided golden-path tour
  startTour: () => void;
  setTour: (n: number) => void;
  goLive: () => void;
  openRun: (name: string, atT?: number) => Promise<void>;
  setT: (t: number) => void;
  togglePlay: () => void;
  tick: () => void;
};

export const useStudio = create<StudioState>((set, get) => ({
  source: new LiveSource(),
  t: Date.now() / 1000,
  playing: true,
  follow: true,
  scopeKey: 0,

  bumpScope: () => set((s) => ({ scopeKey: s.scopeKey + 1 })),

  tourStep: 0,
  startTour: () => set({ tourStep: 1 }),
  setTour: (n) => set({ tourStep: n }),

  goLive: () => {
    get().source.dispose();
    set({ source: new LiveSource(), follow: true, playing: true, t: Date.now() / 1000 });
  },

  openRun: async (name, atT) => {
    get().source.dispose();
    const run = new RunSource(name);
    await run.whenReady();
    const b = run.bounds();
    const t = atT ?? b?.start ?? 0;
    set({ source: run, follow: false, playing: false, t });
  },

  setT: (t) => {
    set({ t, follow: false });
  },

  // live: pause = stop pinning to NOW (freeze playhead). run: pause = stop playback.
  togglePlay: () => set((st) => (st.source.kind === "live" ? { follow: !st.follow } : { playing: !st.playing })),

  // driven by one rAF loop in App; advances the playhead.
  tick: () => {
    const { source, playing, follow, t } = get();
    if (source.kind === "live") {
      if (follow) set({ t: Date.now() / 1000 });
      return;
    }
    if (!playing) return;
    const b = source.bounds();
    const next = t + 0.1; // matches the 100ms tick → realtime replay
    if (b && next >= b.end) {
      set({ t: b.end, playing: false });
      return;
    }
    set({ t: next });
  },
}));
