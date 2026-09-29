"""MQTT transport for analysis/monitoring (device -> laptop).

TCP sockets are reserved for Pi<->Pico control, so monitoring goes over MQTT only.

Topics (all under reverseeng/v1/<device_id>/):
  profile    retained, QoS 1  full device profile JSON
  status     retained, QoS 1  {"state": "online"|"offline"}; offline is the Last Will
  telemetry  QoS 0            live samples from telemetry.sample()
"""

from __future__ import annotations

import json

import paho.mqtt.client as mqtt

TOPIC_ROOT = "reverseeng/v1"


def topic(device_id: str, kind: str) -> str:
    return f"{TOPIC_ROOT}/{device_id}/{kind}"


def make_client(client_id: str) -> mqtt.Client:
    return mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=client_id)


def status_payload(state: str) -> str:
    return json.dumps({"state": state})
