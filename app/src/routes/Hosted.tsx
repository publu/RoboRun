// Folds a still-useful legacy page into Studio's single shell via an iframe
// (shell.js hides its own chrome when framed). One nav, one front door — no
// rewrite of pages that already work. Native ports are a later polish.
export function Hosted({ title, src }: { title: string; src: string }) {
  return (
    <div style={{ display: "flex", flexDirection: "column", height: "calc(100vh - 92px)" }}>
      <iframe title={title} src={src} className="scene-stage" style={{ flex: 1 }} />
    </div>
  );
}
