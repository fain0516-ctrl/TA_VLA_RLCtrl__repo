"""Laptop side: receive profiles, validate them against the schema, show status/telemetry."""

from __future__ import annotations

import json
import sys
import time
from typing import Callable

from .schema import validate_profile
from .transport import TOPIC_ROOT, make_client


def summarize(profile: dict) -> str:
    board = profile.get("board") or {}
    kernel = profile.get("kernel") or {}
    sec = profile.get("security") or {}
    lines = [
        f"device   {profile.get('device_id')}  ({profile.get('collection', {}).get('mode')})",
        f"board    {board.get('model')}  rev {board.get('revision')}",
        f"soc      {(profile.get('chipset') or {}).get('soc')}",
        f"kernel   {kernel.get('release')}",
        f"pins     {len((profile.get('pinout') or {}).get('pins', []))} ({(profile.get('pinout') or {}).get('source')})",
        f"secure   {(sec.get('secure_boot') or {}).get('state')}",
    ]
    for r in sec.get("lock_risks", []):
        lines.append(f"  [{r['severity'].upper()}{' IRREVERSIBLE' if r['irreversible'] else ''}] {r['message']}")
    if profile.get("errors"):
        lines.append(f"errors   {len(profile['errors'])} collector(s) failed: " + ", ".join(e["collector"] for e in profile["errors"]))
    return "\n".join(lines)


def run(host: str, port: int = 1883, device: str = "+", on_event: Callable[[str, str, dict, list[str]], None] | None = None,
        max_messages: int | None = None, timeout: float | None = None) -> None:
    """Subscribe to one device (or all with "+"). Calls on_event(device_id, kind, payload, schema_errors)."""
    def default_event(device_id, kind, payload, errors):
        if kind == "profile":
            print(f"--- profile {'OK' if not errors else 'INVALID'} ---")
            print(summarize(payload))
            for e in errors:
                print(f"  schema: {e}")
        else:
            print(f"{device_id} {kind}: {json.dumps(payload)}")
        sys.stdout.flush()

    handler = on_event or default_event
    received = 0
    client = make_client("reverseeng-laptop")

    def on_connect(c, userdata, flags, reason_code, properties):
        c.subscribe(f"{TOPIC_ROOT}/{device}/#", qos=1)

    def on_message(c, userdata, msg):
        nonlocal received
        _, _, device_id, kind = msg.topic.split("/", 3)
        try:
            payload = json.loads(msg.payload)
        except ValueError:
            payload, errors = {"raw": msg.payload.decode(errors="replace")}, ["payload is not JSON"]
        else:
            errors = validate_profile(payload) if kind == "profile" else []
        handler(device_id, kind, payload, errors)
        received += 1
        if max_messages is not None and received >= max_messages:
            c.disconnect()

    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(host, port, keepalive=30)
    if timeout is None:
        client.loop_forever()
        return
    client.loop_start()
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and (max_messages is None or received < max_messages):
        time.sleep(0.05)
    client.loop_stop()
    client.disconnect()
