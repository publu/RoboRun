"""The ``manifest`` CLI in sim (default). ``--live`` paths are exercised only for their refusals here; the live
drills need a real robot with an operator present."""
import json

import pytest

from roborun_sdk import cli

from conftest import EXAMPLES, FIXTURES


def test_run_segments_in_sim(capsys):
    for name in ("sim-insert.rows30", "sim-reach.path-rows"):
        code = cli.main(["run", str(FIXTURES / f"{name}.segment.json"), "--json"])
        out = json.loads(capsys.readouterr().out)
        assert code == 0, out
        assert out["mode"] == "SIM" and out["ack"]["result"] == "accepted" and out["gate_report"]["outcome"] == "complete"


def test_run_refuses_a_tampered_segment(tmp_path, capsys):
    seg = json.loads((FIXTURES / "sim-insert.rows30.segment.json").read_text())
    seg["limits"]["ee_speed_m_s"] = 0.19
    path = tmp_path / "tampered.json"
    path.write_text(json.dumps(seg))
    code = cli.main(["run", str(path), "--json"])
    out = json.loads(capsys.readouterr().out)
    assert code == 2 and out["ack"]["reason"] == "bad_signature"


def test_run_from_current_in_sim(capsys):
    code = cli.main(["run", "--from-current", "left", "--json"])
    out = json.loads(capsys.readouterr().out)
    assert code == 0 and out["ack"]["result"] == "accepted" and out["gate_report"]["outcome"] == "complete"


def test_live_run_requires_keys():
    with pytest.raises(SystemExit) as exc:
        cli.main(["run", str(FIXTURES / "sim-insert.rows30.segment.json"), "--live", "--geometry", "capsule"])
    assert "--root-keys" in str(exc.value)


def test_stop_sim_contacts_no_robot(capsys):
    code = cli.main(["stop", "--sim", "--json"])
    out = json.loads(capsys.readouterr().out)
    assert code == 0 and out["mode"].startswith("SIM") and out["report"]["not_confirmed"] == []


def test_doctor_sim(capsys):
    code = cli.main(["doctor", "--json"])
    out = json.loads(capsys.readouterr().out)
    assert code == 0, out
    assert out["import_guard"]["ok"] and out["sim_selftest"]["ok"] and "live" not in out


def test_rt_process_serves_stop_and_socket(tmp_path):
    """`roborun rt` (sim) in its own interpreter: guard installed, heap frozen, UDP Stop and the link socket work."""
    import os
    import subprocess
    import sys
    import time

    from roborun_sdk.link.rtclient import RtClient
    from roborun_sdk.rt.stop_listeners import send_udp_stop
    src = str(EXAMPLES.parent / "src")
    sock = str(tmp_path / "rt.sock")
    env = dict(os.environ, PYTHONPATH=src, OPENBLAS_NUM_THREADS="1", OMP_NUM_THREADS="1")
    proc = subprocess.Popen([sys.executable, "-m", "roborun_sdk.cli", "rt", "--seconds", "3", "--stop-port", "0",
                             "--socket", sock], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, env=env)
    try:
        first = json.loads(proc.stdout.readline())
        assert first["manifest_rt"] == "running" and first["forbidden_modules"] == []
        client = RtClient(sock)
        info = client.info()
        assert set(info["arms"]) == {"left", "right"} and info["latched"]["stopped"] is False
        send_udp_stop("127.0.0.1", first["stop_udp"], "stp_cli_test", repeats=1)
        deadline = time.time() + 2
        while not client.info()["latched"]["stopped"] and time.time() < deadline:
            time.sleep(0.05)
        assert client.info()["latched"]["stopped"] is True
        kinds = [e.get("kind") for e in client.events(0)]
        assert "stopped" in kinds
        assert first["console_token"] and first["refused_kids"] == []          # sim rt: DEV keys are fine
        assert cli.main(["clear-stop", "--token", "wrong", "--operator", "operator", "--socket", sock, "--json"]) == 2
        assert client.info()["latched"]["stopped"] is True                    # no console token: still latched
        assert cli.main(["clear-stop", "--token", first["console_token"], "--operator", "operator", "--socket", sock,
                         "--json"]) == 0
        assert client.info()["latched"]["stopped"] is False                   # cleared by the local operator
        client.close()
        out, err = proc.communicate(timeout=10)
        last = json.loads(out.strip().splitlines()[-1])
        assert proc.returncode == 0 and last["manifest_rt"] == "exited" and last["stops"] >= 1 and last["ticks"] > 30
    finally:
        if proc.poll() is None:
            proc.kill()
