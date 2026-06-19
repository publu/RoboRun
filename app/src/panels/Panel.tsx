import type { ReactNode } from "react";

export function Panel({ title, children }: { title: string; children: ReactNode }) {
  return (
    <div className="panel">
      <div className="panel-head">{title}</div>
      <div className="panel-body">{children}</div>
    </div>
  );
}
