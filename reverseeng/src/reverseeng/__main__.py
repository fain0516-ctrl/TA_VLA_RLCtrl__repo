"""CLI: python -m reverseeng {collect,publish,subscribe}"""

from __future__ import annotations

import argparse
import json
import sys

from .agent import build_profile
from .schema import validate_profile
from .source import Source


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(prog="reverseeng", description="Read-only device profiling over MQTT.")
    sub = p.add_subparsers(dest="cmd", required=True)

    def add_source_args(sp):
        sp.add_argument("--root", default="/", help="filesystem root to read (a mounted SD card for offline mode)")
        sp.add_argument("--mode", choices=["running", "offline"], default=None,
                        help="default: running when --root is /, otherwise offline")

    c = sub.add_parser("collect", help="build a profile and print or save it")
    add_source_args(c)
    c.add_argument("-o", "--output", help="write JSON here instead of stdout")

    pub = sub.add_parser("publish", help="collect, publish retained profile, stream telemetry")
    add_source_args(pub)
    pub.add_argument("--broker", required=True)
    pub.add_argument("--port", type=int, default=1883)
    pub.add_argument("--interval", type=float, default=5.0, help="telemetry period in seconds")
    pub.add_argument("--count", type=int, default=None, help="stop after N telemetry samples")

    s = sub.add_parser("subscribe", help="laptop side: receive and validate profiles")
    s.add_argument("--broker", required=True)
    s.add_argument("--port", type=int, default=1883)
    s.add_argument("--device", default="+", help="device id, or + for all")

    args = p.parse_args(argv)

    if args.cmd == "subscribe":
        from . import subscriber
        subscriber.run(args.broker, args.port, args.device)
        return 0

    mode = args.mode or ("running" if args.root == "/" else "offline")
    src = Source(args.root)
    profile = build_profile(src, mode)
    errors = validate_profile(profile)
    for e in errors:
        print(f"schema: {e}", file=sys.stderr)

    if args.cmd == "collect":
        text = json.dumps(profile, indent=2, ensure_ascii=False)
        if args.output:
            with open(args.output, "w") as f:
                f.write(text + "\n")
        else:
            print(text)
        return 1 if errors else 0

    from . import publisher
    publisher.run(profile, src, args.broker, args.port, args.interval, args.count)
    return 0


if __name__ == "__main__":
    sys.exit(main())
