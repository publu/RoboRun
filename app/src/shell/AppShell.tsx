import { NavLink, Outlet, useLocation } from "react-router-dom";
import { Scrubber } from "./Scrubber";
import { RecordButton } from "./RecordButton";
import { ScopeSwitcher } from "./ScopeSwitcher";
import { ErrorBoundary } from "./ErrorBoundary";
import { TourOverlay } from "./TourOverlay";
import { useStudio } from "../store";
import "./shell.css";

// Two clear axes: WHAT you're doing (Live now / Replay a run / Search history)
// and WHERE (a sim or robot you launch under Build). The project scope in the
// header threads through all of it.
// Priority-flat: the four you use constantly sit at top with no label; the
// occasional tools drop below a divider, so position signals usefulness.
type NavItem = { icon: string; label: string; to: string; hint: string };
type NavSection = { group?: string; divider?: boolean; items: NavItem[] };
const NAV: NavSection[] = [
  {
    items: [
      { icon: "◉", label: "Live", to: "/live", hint: "what's happening now" },
      { icon: "▣", label: "Sims", to: "/sims", hint: "arena · fleet · data sim" },
      { icon: "⊞", label: "Runs", to: "/runs", hint: "replay recorded runs" },
      { icon: "⌕", label: "Search", to: "/search", hint: "find anything seen" },
    ],
  },
  {
    group: "More",
    divider: true,
    items: [
      { icon: "◆", label: "Agent", to: "/agent", hint: "drive the robot with an AI agent (MCP)" },
      { icon: "▤", label: "Analytics", to: "/analytics", hint: "fleet-wide stats" },
      { icon: "✦", label: "Scenarios", to: "/scenarios", hint: "scored behavior tests" },
      { icon: "◈", label: "Swarm Lab", to: "/swarm", hint: "fleet coordination strategies" },
    ],
  },
];

export function AppShell() {
  const scopeKey = useStudio((s) => s.scopeKey);
  const { pathname } = useLocation();
  // the playhead/scrubber only means something where there's a timeline to
  // follow or scrub — Live and Runs. Elsewhere it's just confusing chrome.
  const showScrubber = pathname.endsWith("/live") || pathname.endsWith("/runs");
  return (
    <div className="app-shell">
      <aside className="app-side">
        <a className="app-brand" href="/studio/live">
          <span className="dia">◇</span>
          <span>
            <b>RoboRun</b>
          </span>
          <span className="sub">studio</span>
        </a>
        <nav className="app-nav">
          {NAV.map((sec, i) => (
            <div key={sec.group ?? i}>
              {sec.divider && <div className="nav-sep" />}
              {sec.group && <div className="nav-group">{sec.group}</div>}
              {sec.items.map((it) => (
                <NavLink key={it.to} to={it.to} title={it.hint} className={({ isActive }) => "nav-item" + (isActive ? " on" : "")}>
                  <span className="ni-ic">{it.icon}</span>
                  <span className="ni-l">{it.label}</span>
                </NavLink>
              ))}
            </div>
          ))}
        </nav>
        <div className="app-side-foot">every run · sealed + searchable</div>
      </aside>
      <div className="app-col">
        <header className="app-top">
          <div className="app-title">STUDIO</div>
          <div className="app-top-r">
            <ScopeSwitcher />
            <RecordButton />
          </div>
        </header>
        {showScrubber && <Scrubber />}
        <div className="app-main">
          {/* keyed on scopeKey: switching project remounts only the content
              (refetching scoped data) — the shell stays put, no full reload */}
          <ErrorBoundary key={scopeKey}>
            <Outlet />
          </ErrorBoundary>
        </div>
        <TourOverlay />
      </div>
    </div>
  );
}
