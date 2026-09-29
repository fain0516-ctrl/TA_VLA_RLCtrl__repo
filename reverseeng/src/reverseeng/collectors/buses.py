from __future__ import annotations

from ..source import Source


def _attr(src: Source, path: str) -> str | None:
    v = src.read_text(path)
    return v.strip() if v is not None else None


def collect_buses(src: Source) -> dict:
    i2c = [
        {"adapter": d, "name": _attr(src, f"/sys/bus/i2c/devices/{d}/name")}
        for d in src.listdir("/sys/bus/i2c/devices")
        if d.startswith("i2c-")
    ]

    usb = []
    for d in src.listdir("/sys/bus/usb/devices"):
        base = f"/sys/bus/usb/devices/{d}"
        vid, pid = _attr(src, f"{base}/idVendor"), _attr(src, f"{base}/idProduct")
        if vid and pid:  # interfaces (e.g. 1-1:1.0) have no ids
            usb.append({
                "path": d,
                "vendor_id": vid,
                "product_id": pid,
                "manufacturer": _attr(src, f"{base}/manufacturer"),
                "product": _attr(src, f"{base}/product"),
            })

    pcie = []
    for d in src.listdir("/sys/bus/pci/devices"):
        base = f"/sys/bus/pci/devices/{d}"
        vid, did = _attr(src, f"{base}/vendor"), _attr(src, f"{base}/device")
        if vid and did:
            pcie.append({"address": d, "vendor_id": vid, "device_id": did, "class": _attr(src, f"{base}/class")})

    return {"i2c": i2c, "usb": usb, "pcie": pcie}
