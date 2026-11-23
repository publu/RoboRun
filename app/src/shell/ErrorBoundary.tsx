import { Component, type ReactNode } from "react";

// One panel throwing must not blank the whole studio. Catches render errors and
// shows an inline, dismissable fallback so the rest of the UI stays usable.
export class ErrorBoundary extends Component<{ children: ReactNode }, { err?: Error }> {
  state: { err?: Error } = {};

  static getDerivedStateFromError(err: Error) {
    return { err };
  }

  render() {
    if (this.state.err) {
      return (
        <div className="panel" style={{ margin: 14 }}>
          <div className="panel-head" style={{ color: "var(--bad)" }}>
            panel error
          </div>
          <div className="panel-body" style={{ fontFamily: "var(--mono)", fontSize: 12 }}>
            <div style={{ color: "var(--bad)", marginBottom: 8 }}>{this.state.err.message}</div>
            <button className="btn sm" onClick={() => this.setState({ err: undefined })}>
              retry
            </button>
          </div>
        </div>
      );
    }
    return this.props.children;
  }
}
