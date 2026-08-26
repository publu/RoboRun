import { useEffect, useRef, useState } from "react";
import { useStudio } from "../store";

// The spine: the active project/environment scopes Live, Runs and Search. Ported
// from web/shell.js. Switching bumps scopeKey so the content refetches (no full
// page reload / flash).
type Active = { project: string; environment: string } | null;
type Project = { id: string; name: string };
type Env = { id: string; name: string; backend?: string; mode?: string };

export function ScopeSwitcher() {
  const bumpScope = useStudio((s) => s.bumpScope);
  const [open, setOpen] = useState(false);
  const [active, setActive] = useState<Active>(null);
  const [projects, setProjects] = useState<Project[]>([]);
  const [envs, setEnvs] = useState<Record<string, Env[]>>({});
  const ref = useRef<HTMLDivElement>(null);

  const refresh = () =>
    fetch("/api/projects/active")
      .then((r) => r.json())
      .then((a) => setActive(a.active || null))
      .catch(() => {});

  useEffect(() => {
    refresh();
  }, []);

  // close on outside click (not mouse-leave — that made the menu unreachable)
  useEffect(() => {
    if (!open) return;
    const onDown = (e: MouseEvent) => {
      if (ref.current && !ref.current.contains(e.target as Node)) setOpen(false);
    };
    document.addEventListener("mousedown", onDown);
    return () => document.removeEventListener("mousedown", onDown);
  }, [open]);

  const load = async () => {
    const d = await (await fetch("/api/projects")).json().catch(() => ({ projects: [] }));
    const ps: Project[] = d.projects || [];
    setProjects(ps);
    const map: Record<string, Env[]> = {};
    await Promise.all(
      ps.map(async (p) => {
        const e = await (await fetch("/api/environments?project=" + encodeURIComponent(p.id))).json().catch(() => ({}));
        map[p.id] = e.environments || [];
      })
    );
    setEnvs(map);
  };

  const toggle = () => {
    if (!open) load();
    setOpen(!open);
  };

  const apply = async () => {
    setOpen(false);
    await refresh();
    bumpScope(); // remount content to refetch scoped data — no full reload
  };

  const pick = async (project: string, environment: string) => {
    await fetch("/api/projects/active", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ project, environment }),
    });
    await apply();
  };

  const clear = async () => {
    await fetch("/api/projects/active/clear", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: "{}",
    });
    await apply();
  };

  // create a project (+ its default environment) and switch to it
  const createProject = async () => {
    const name = window.prompt("Name your project (groups its runs, robots & data):");
    if (!name || !name.trim()) return;
    const d = await (await fetch("/api/projects", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ name: name.trim() }),
    })).json();
    if (d.ok && d.project?.id) {
      await fetch("/api/projects/active", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ project: d.project.id, environment: "default" }),
      });
      await load();
      await apply();
    }
  };

  // Always visible: it's how you create/switch a project (the scope for all data).
  return (
    <div className="scope" ref={ref}>
      <button className={"scope-btn" + (active ? " scoped" : "")} onClick={toggle}
        title="project / environment — scopes all data. Create or switch here.">
        <span className="dot" />
        {active ? (
          <span>
            {active.project} <span className="lbl-dim">/ {active.environment}</span>
          </span>
        ) : (
          <span className="lbl-dim">Workspace</span>
        )}
        <span className="lbl-dim">▾</span>
      </button>
      {open && (
        <div className="scope-menu">
          <div className="scope-env" onClick={createProject} style={{ color: "var(--accent)" }}>
            <span>＋ New project</span>
          </div>
          {projects.length === 0 && (
            <div className="scope-empty">
              No projects yet — you're in <b>scratch</b> (a throwaway workspace). Create one to group its
              runs, robots and data.
            </div>
          )}
          {projects.map((p) => (
            <div key={p.id}>
              <div className="scope-proj">{p.name}</div>
              {(envs[p.id] || []).map((e) => {
                const on = active?.project === p.id && active?.environment === e.id;
                return (
                  <div key={e.id} className={"scope-env" + (on ? " on" : "")} onClick={() => pick(p.id, e.id)}>
                    <span>{e.name}</span>
                    <span className="se-b">
                      {e.backend}·{e.mode}
                    </span>
                  </div>
                );
              })}
            </div>
          ))}
          <div className="scope-acts">
            <a href="/studio/sims">+ new sim / robot</a>
            <a onClick={clear}>use scratch</a>
          </div>
        </div>
      )}
    </div>
  );
}
