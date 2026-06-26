import { useEffect } from "react";
import { BrowserRouter, Routes, Route } from "react-router-dom";
import { AppShell } from "./shell/AppShell";
import { useStudio } from "./store";
import { Home } from "./routes/Home";
import { Live } from "./routes/Live";
import { Runs } from "./routes/Runs";
import { Sims } from "./routes/Sims";
import { Search } from "./routes/Search";
import { Scenarios } from "./routes/Scenarios";
import { Analytics } from "./routes/Analytics";
import { Agent } from "./routes/Agent";
import { Hosted } from "./routes/Hosted";

// One throttled loop drives the playhead. 10Hz is plenty — the camera polls at
// 5Hz and events are coarse — and it avoids 60fps whole-app re-renders.
function usePlayhead() {
  const tick = useStudio((s) => s.tick);
  useEffect(() => {
    const id = setInterval(tick, 100);
    return () => clearInterval(id);
  }, [tick]);
}

export function App() {
  usePlayhead();
  return (
    <BrowserRouter basename="/studio">
      <Routes>
        <Route element={<AppShell />}>
          <Route index element={<Home />} />
          <Route path="live" element={<Live />} />
          <Route path="sims" element={<Sims />} />
          <Route path="runs" element={<Runs />} />
          <Route path="search" element={<Search />} />
          <Route path="agent" element={<Agent />} />
          <Route path="swarm" element={<Hosted title="Swarm Lab" src="/fleet" />} />
          <Route path="scenarios" element={<Scenarios />} />
          <Route path="analytics" element={<Analytics />} />
        </Route>
      </Routes>
    </BrowserRouter>
  );
}
