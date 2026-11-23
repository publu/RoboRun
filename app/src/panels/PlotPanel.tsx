import { useEffect, useRef, useState } from "react";
import uPlot from "uplot";
import "uplot/dist/uPlot.min.css";
import { useStudio } from "../store";
import type { Sample } from "../source/Source";
import { Panel } from "./Panel";

const COLORS = ["#00d47e", "#4090e0", "#d4a030", "#e0563f"];

// Plots every numeric field of a telemetry channel. Source-agnostic: live pulls
// the WS ring buffer, replay pulls run_series' named arrays.
export function PlotPanel({ channel }: { channel: string }) {
  const source = useStudio((s) => s.source);
  const host = useRef<HTMLDivElement>(null);
  const plot = useRef<uPlot | undefined>(undefined);
  const [empty, setEmpty] = useState(true);

  useEffect(() => {
    let alive = true;
    const draw = async () => {
      const rows: Sample[] = await source.series(channel);
      if (!alive || !host.current) return;
      setEmpty(rows.length === 0);
      const fields = [...new Set(rows.flatMap((r) => Object.keys(r)))].filter((k) => k !== "t");
      const xs = rows.map((r) => r.t);
      const data: uPlot.AlignedData = [xs, ...fields.map((f) => rows.map((r) => r[f] ?? null))];
      if (!plot.current) {
        const series: uPlot.Series[] = [
          {},
          ...fields.map((f, i) => ({ label: f, stroke: COLORS[i % COLORS.length], width: 1.5 })),
        ];
        plot.current = new uPlot(
          {
            width: host.current.clientWidth || 360,
            height: 160,
            series,
            axes: [
              { stroke: "#6f8a78", grid: { stroke: "#212c22" } },
              { stroke: "#6f8a78", grid: { stroke: "#212c22" } },
            ],
          },
          data,
          host.current
        );
      } else {
        plot.current.setData(data);
      }
    };
    draw();
    const iv = source.kind === "live" ? setInterval(draw, 500) : 0;
    return () => {
      alive = false;
      if (iv) clearInterval(iv);
      plot.current?.destroy();
      plot.current = undefined;
    };
  }, [source, channel]);

  return (
    <Panel title={`Telemetry · ${channel}`}>
      <div style={{ position: "relative" }}>
        <div ref={host} style={{ opacity: empty ? 0 : 1 }} />
        {empty && (
          <div style={{ color: "var(--fg-dim)", fontSize: 12, padding: "28px 0", textAlign: "center" }}>
            {source.kind === "live" ? "No telemetry yet — start a sim or connect a robot." : "No telemetry recorded in this run."}
          </div>
        )}
      </div>
    </Panel>
  );
}
