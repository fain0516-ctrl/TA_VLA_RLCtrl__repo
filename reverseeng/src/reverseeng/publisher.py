"""Device side: publish the retained profile, then stream telemetry."""

from __future__ import annotations

import json
import time

from . import telemetry
from .source import Source
from .transport import make_client, status_payload, topic


def run(profile: dict, src: Source, host: str, port: int = 1883, interval: float = 5.0, count: int | None = None) -> None:
    device_id = profile["device_id"]
    client = make_client(f"reverseeng-agent-{device_id}")
    client.will_set(topic(device_id, "status"), status_payload("offline"), qos=1, retain=True)
    client.connect(host, port, keepalive=30)
    client.loop_start()
    try:
        client.publish(topic(device_id, "status"), status_payload("online"), qos=1, retain=True)
        client.publish(topic(device_id, "profile"), json.dumps(profile), qos=1, retain=True).wait_for_publish(10)
        sent = 0
        while count is None or sent < count:
            client.publish(topic(device_id, "telemetry"), json.dumps(telemetry.sample(src)), qos=0)
            sent += 1
            if count is None or sent < count:
                time.sleep(interval)
        # Orderly exit: mark offline ourselves (the Will only fires on a dropped connection).
        client.publish(topic(device_id, "status"), status_payload("offline"), qos=1, retain=True).wait_for_publish(10)
    finally:
        client.loop_stop()
        client.disconnect()
