"""Strata CLI — `strata serve` plus a few admin helpers.

    strata serve --store s3://bucket/prefix?endpoint=http://localhost:9000 --port 8420
    strata serve --store ./data            # local object store (dev)
    strata info  --store ./data
    strata token create ingest --write '*' --store ./data
"""
from __future__ import annotations

import argparse
import json
import sys

from . import Strata, __version__


def _store_parent():
    p = argparse.ArgumentParser(add_help=False)
    p.add_argument("--store", default="./strata-data",
                   help="object store URL: file path, or s3://bucket/prefix?endpoint=...")
    return p


def cmd_serve(a):
    from .server import StrataServer
    srv = StrataServer(a.store, host=a.host, port=a.port)
    print(f"strata {__version__} · http://{a.host}:{srv.port} · store={a.store}", flush=True)
    print(f"  auth: {srv.db.info()['auth']}  ·  models: blobs + vectors", flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\nshutting down…", flush=True)
        srv.shutdown()


def cmd_info(a):
    print(json.dumps(Strata(a.store).info(), indent=2))


def cmd_token(a):
    db = Strata(a.store)
    if a.token_cmd == "create":
        perms = {"full_access": a.full,
                 "read": (a.read or "").split(",") if a.read else [],
                 "write": (a.write or "").split(",") if a.write else []}
        value = db.tokens.create(a.name, perms)
        print(value)
    elif a.token_cmd == "list":
        print(json.dumps(db.tokens.list(), indent=2))
    elif a.token_cmd == "remove":
        db.tokens.remove(a.name)
        print(f"removed {a.name}")


def main(argv=None):
    common = _store_parent()
    p = argparse.ArgumentParser(prog="strata",
                                description="Object-storage-native time-series + vector engine")
    p.add_argument("--version", action="version", version=f"strata {__version__}")
    sub = p.add_subparsers(dest="cmd", required=True)

    s = sub.add_parser("serve", help="run the HTTP server", parents=[common])
    s.add_argument("--host", default="127.0.0.1")
    s.add_argument("--port", type=int, default=8420)
    s.set_defaults(fn=cmd_serve)

    i = sub.add_parser("info", help="print engine info", parents=[common])
    i.set_defaults(fn=cmd_info)

    t = sub.add_parser("token", help="manage API tokens")
    tsub = t.add_subparsers(dest="token_cmd", required=True)
    tc = tsub.add_parser("create", parents=[common]); tc.add_argument("name")
    tc.add_argument("--full", action="store_true"); tc.add_argument("--read"); tc.add_argument("--write")
    tsub.add_parser("list", parents=[common])
    tr = tsub.add_parser("remove", parents=[common]); tr.add_argument("name")
    t.set_defaults(fn=cmd_token)

    a = p.parse_args(argv)
    return a.fn(a)


if __name__ == "__main__":
    sys.exit(main())
