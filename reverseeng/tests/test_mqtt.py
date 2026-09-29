"""End-to-end over a real broker: device publishes, laptop subscribes and validates.

Skipped when mosquitto is not installed.
"""

import os
import shutil
import socket
import subprocess
import time

import pytest

from reverseeng import publisher, subscriber
from reverseeng.agent import build_profile
from reverseeng.source import Source

MOSQUITTO = shutil.which("mosquitto") or next((p for p in ("/usr/sbin/mosquitto",) if os.path.exists(p)), None)
pytestmark = pytest.mark.skipif(MOSQUITTO is None, reason="mosquitto not installed")


@pytest.fixture
def broker(tmp_path):
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]
    conf = tmp_path / "mosquitto.conf"
    conf.write_text(f"listener {port} 127.0.0.1\nallow_anonymous true\npersistence false\n")
    proc = subprocess.Popen([MOSQUITTO, "-c", str(conf)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    for _ in range(50):
        try:
            socket.create_connection(("127.0.0.1", port), timeout=0.1).close()
            break
        except OSError:
            time.sleep(0.1)
    yield port
    proc.terminate()
    proc.wait(5)


def test_retained_profile_reaches_late_subscriber(pi5_root, broker):
    src = Source(pi5_root)
    profile = build_profile(src)
    publisher.run(profile, src, "127.0.0.1", broker, interval=0, count=2)

    # Subscriber connects after the device is done: retained profile + final status still arrive.
    events = []
    subscriber.run("127.0.0.1", broker, on_event=lambda *e: events.append(e), max_messages=2, timeout=5)
    by_kind = {kind: (dev, payload, errors) for dev, kind, payload, errors in events}
    assert set(by_kind) == {"profile", "status"}
    dev, payload, errors = by_kind["profile"]
    assert dev == "abcdef12" and errors == [] and payload["chipset"]["soc"] == "bcm2712"
    assert by_kind["status"][1] == {"state": "offline"}


def test_live_telemetry_stream(pi5_root, broker):
    src = Source(pi5_root)
    profile = build_profile(src)
    events = []

    import threading
    t = threading.Thread(target=subscriber.run, kwargs=dict(
        host="127.0.0.1", port=broker, on_event=lambda *e: events.append(e), max_messages=5, timeout=5))
    t.start()
    time.sleep(0.5)
    publisher.run(profile, src, "127.0.0.1", broker, interval=0.05, count=3)
    t.join()
    telemetry = [p for _, kind, p, _ in events if kind == "telemetry"]
    assert len(telemetry) == 3
    assert telemetry[0]["cpu_temp_c"] == 48.25 and telemetry[0]["uptime_s"] == 3600.5
