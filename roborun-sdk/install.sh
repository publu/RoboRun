#!/bin/sh
# Roborun installer:  curl -fsSL https://roborun.app/install.sh | sh
#
# Installs into ~/.roborun with its own Python, without administrator rights, and puts `roborun` on your PATH. (The simulator
# needs one system OpenGL package, and USB arms one permission; Roborun prints the exact command when they are missing.)
# Running it again updates Roborun. To remove it: roborun uninstall
#
# ROBORUN_VERSION=V      install that release (roborun --version names it, e.g. 0.1.20261119172831) instead of the latest
# ROBORUN_BUILD=ID       install that build instead of the latest
# ROBORUN_WHEELS=DIR      also look for the packages in DIR
# ROBORUN_NO_MODIFY_PATH=1  leave your shell settings alone
set -eu

main() {
    say() { printf '%s\n' "$*"; }
    die() { printf 'roborun install: %s\n' "$*" >&2; exit 1; }

    case "$(uname -s)" in
        Darwin|Linux) ;;
        *) die "Roborun runs on macOS and Linux. On Windows, install WSL2 with Ubuntu and run this there." ;;
    esac
    [ -n "${HOME:-}" ] || die "HOME is not set"
    command -v curl >/dev/null 2>&1 || die "curl is required"

    root="$HOME/.roborun"
    tools="$root/tools"
    venv="$root/python"
    bin="$HOME/.local/bin"
    python_version=3.12
    mkdir -p "$tools" "$bin"

    if [ ! -x "$tools/uv" ]; then
        say "Getting the Python installer..."
        uv_script="$(mktemp)"
        # downloaded first, so a failed download is reported as one (piped into sh, curl's failure would be lost)
        if ! curl -fsSL --retry 2 https://astral.sh/uv/install.sh -o "$uv_script"; then
            rm -f "$uv_script"
            die "could not reach astral.sh to download the Python installer: check the internet connection, or the proxy / firewall"
        fi
        env UV_UNMANAGED_INSTALL="$tools" INSTALLER_PRINT_QUIET=1 sh "$uv_script" >/dev/null || { rm -f "$uv_script"; die "could not install the Python installer"; }
        rm -f "$uv_script"
        [ -x "$tools/uv" ] || die "the Python installer did not install (no $tools/uv)"
    fi
    uv="$tools/uv"
    export UV_PYTHON_INSTALL_DIR="$root/pythons" UV_CACHE_DIR="$root/cache"

    if [ ! -x "$venv/bin/python" ]; then
        say "Setting up Python $python_version..."
        "$uv" venv --quiet --seed --python "$python_version" --python-preference only-managed "$venv" \
            || die "could not set up Python"
    fi

    say "Installing Roborun..."
    if [ -n "${ROBORUN_WHEELS:-}" ]; then
        "$uv" pip install --quiet --python "$venv/bin/python" --upgrade --find-links "$ROBORUN_WHEELS" \
            --reinstall-package roborun-sdk --reinstall-package roborun-station 'roborun-sdk[robot]' \
            || die "could not install Roborun"
    else
        base="${ROBORUN_DOWNLOADS:-https://roborun.app/dl}"
        work="$(mktemp -d)"
        trap 'rm -rf "$work"' EXIT
        if [ -n "${ROBORUN_VERSION:-}" ] && [ -z "${ROBORUN_BUILD:-}" ]; then
            curl -fsSL "$base/index.json" -o "$work/index.json" || die "could not read the list of Roborun releases at $base/index.json"
            ROBORUN_BUILD="$("$venv/bin/python" - "$work/index.json" "$ROBORUN_VERSION" <<'PY'
import json, sys
rows = json.load(open(sys.argv[1])).get('releases', [])
found = [r['build'] for r in rows if r.get('version') == sys.argv[2]]
if not found:
    raise SystemExit('no Roborun release ' + sys.argv[2] + '. Releases: ' + ', '.join(r.get('version', '?') for r in rows[-5:]))
print(found[-1])
PY
)" || die "ROBORUN_VERSION=$ROBORUN_VERSION is not a Roborun release"
        fi
        if [ -n "${ROBORUN_BUILD:-}" ]; then manifest="$base/$ROBORUN_BUILD/manifest.json"; else manifest="$base/latest.json"; fi
        if ! curl -fsSL --retry 2 "$manifest" -o "$work/manifest.json"; then
            curl -fsSI "$base/index.json" >/dev/null 2>&1 \
                || die "could not reach ${base%/dl}: check the internet connection, or the proxy / firewall"
            die "no Roborun release is published at $manifest yet. If you were given a build id: ROBORUN_BUILD=ID sh install.sh"
        fi
        wheels="$("$venv/bin/python" - "$work" "$base" <<'PY'
import hashlib, json, subprocess, sys
from pathlib import Path
work, base = Path(sys.argv[1]), sys.argv[2]
m = json.loads((work / 'manifest.json').read_text())
for row in m['wheels'].values():
    name = Path(row['file']).name
    target = work / name
    subprocess.run(['curl', '-fsSL', f"{base}/{m['build']}/{name}", '-o', str(target)], check=True)
    if hashlib.sha256(target.read_bytes()).hexdigest() != row['sha256']:
        raise SystemExit(f'{name} failed its checksum')
    # the SDK with its [robot] extra: everything a robot runs (simulator, cameras, arms); a plain pip install is the light SDK
    print(f'{target}[robot]' if name.startswith('roborun_sdk-') else target)
PY
)" || die "could not download Roborun"
        set -f                                             # the SDK wheel ends in [robot]: split the list, never glob it
        # shellcheck disable=SC2086
        "$uv" pip install --quiet --python "$venv/bin/python" --upgrade \
            --reinstall-package roborun-sdk --reinstall-package roborun-station $wheels || die "could not install Roborun"
        set +f
        # which release this is (beside the Python, e.g. ~/.roborun/build.json): roborun --version and the User-Agent name it
        cp "$work/manifest.json" "$(dirname "$venv")/build.json" 2>/dev/null || true
    fi

    ln -sf "$venv/bin/roborun" "$bin/roborun"
    "$bin/roborun" --help >/dev/null 2>&1 || die "Roborun was installed but does not start"
    # the guide an AI assistant reads to run Roborun for you, beside the install
    "$bin/roborun" docs agents > "$root/AGENTS.md" 2>/dev/null || rm -f "$root/AGENTS.md"

    on_path=no
    case ":$PATH:" in *":$bin:"*) on_path=yes ;; esac
    if [ "$on_path" = no ] && [ -z "${ROBORUN_NO_MODIFY_PATH:-}" ]; then
        case "${SHELL:-}" in
            */zsh) rc="$HOME/.zshrc" ;;
            */bash) if [ "$(uname -s)" = Darwin ]; then rc="$HOME/.bash_profile"; else rc="$HOME/.bashrc"; fi ;;
            *) rc="$HOME/.profile" ;;
        esac
        line='export PATH="$HOME/.local/bin:$PATH"'
        if ! grep -qsF "$line" "$rc"; then
            printf '\n# Roborun\n%s\n' "$line" >> "$rc"
        fi
        say ""
        say "Open a new terminal, or run:  $line"
    fi

    [ -n "${ROBORUN_UPGRADING:-}" ] && return 0          # `roborun upgrade` says what happens next itself

    if [ -z "${ROBORUN_FROM_CONNECT:-}" ]; then            # connect.sh says its own next steps (or starts Roborun): each line once
    say ""
    say "Roborun is installed. Next:"
    say "  1. roborun login                  sign this computer in: approve the link it prints"
    say "  2. roborun up --sim cup           no arms: two simulated YAM arms"
    say "     roborun up                     your arms and cameras plugged in: a guided setup the first time"
    say "  3. open https://app.roborun.app on this computer, or this robot's own viewer: roborun view"
    say "AI assistant? Have it read: roborun docs agents   ($root/AGENTS.md)"
    say "Roborun is invite-only: if you don't have a key, email support@roborun.dev"
    fi
    # the same check `roborun up --sim` makes (roborun_sdk.robot.gl_env), so the two never disagree
    if ! "$venv/bin/python" -c 'from roborun_sdk.robot import gl_env; gl_env()' >/dev/null 2>&1; then
        say ""
        say "The simulator also needs an OpenGL library (this one needs an administrator):"
        say "  sudo apt install libosmesa6      (Ubuntu/Debian; Fedora: sudo dnf install mesa-libOSMesa)"
    fi
    if [ -f "$HOME/.config/roborun/robot.json" ] && curl -fsS -m 2 http://127.0.0.1:8120/v1/health >/dev/null 2>&1; then
        say ""
        say "Your robot is still running the version it started with: restart it on this one with  roborun up"
    fi
}

main "$@"
