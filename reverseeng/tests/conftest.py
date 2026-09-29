"""A fake Raspberry Pi 5 filesystem, so collectors run without hardware."""

from pathlib import Path

import pytest

CPUINFO = "".join(
    f"processor\t: {i}\nBogoMIPS\t: 108.00\nCPU architecture: 8\nCPU part\t: 0xd0b\n\n" for i in range(4)
) + "Revision\t: d04170\nSerial\t\t: 10000000abcdef12\nModel\t\t: Raspberry Pi 5 Model B Rev 1.0\n"

CONFIG_TXT = """\
# comment
dtparam=audio=on
camera_auto_detect=1
dtoverlay=vc4-kms-v3d
[pi5]
dtoverlay=i2c-rtc,ds3231
[all]
arm_64bit=1
"""


def write(root: Path, rel: str, data):
    p = root / rel.lstrip("/")
    p.parent.mkdir(parents=True, exist_ok=True)
    (p.write_bytes if isinstance(data, bytes) else p.write_text)(data)


@pytest.fixture
def pi5_root(tmp_path):
    r = tmp_path
    write(r, "/etc/hostname", "pi5-lab\n")
    write(r, "/proc/cpuinfo", CPUINFO)
    write(r, "/proc/meminfo", "MemTotal:        8245068 kB\nMemAvailable:    7000000 kB\n")
    write(r, "/proc/device-tree/model", b"Raspberry Pi 5 Model B Rev 1.0\0")
    write(r, "/proc/device-tree/compatible", b"raspberrypi,5-model-b\0brcm,bcm2712\0")
    write(r, "/proc/device-tree/serial-number", b"abcdef12\0")
    write(r, "/proc/device-tree/chosen/bootloader/build-timestamp", (1700000000).to_bytes(4, "big"))
    write(r, "/proc/device-tree/chosen/bootloader/signed", (0).to_bytes(4, "big"))
    write(r, "/proc/sys/kernel/osrelease", "6.6.51+rpt-rpi-2712\n")
    write(r, "/proc/version", "Linux version 6.6.51+rpt-rpi-2712 (gcc 12.2.0)\n")
    write(r, "/proc/cmdline", "console=serial0,115200 root=PARTUUID=abcd-02 rootfstype=ext4\n")
    write(r, "/proc/modules", "i2c_dev 16384 0 - Live 0x0\nrp1_pio 20480 1 pwm_pio, Live 0x0\n")
    write(r, "/lib/modules/6.6.51+rpt-rpi-2712/modules.dep", "")
    write(r, "/proc/partitions",
          "major minor  #blocks  name\n\n 179        0   31166976 mmcblk0\n 179        1     524288 mmcblk0p1\n"
          " 179        2   30638080 mmcblk0p2\n   1        0       4096 ram0\n")
    write(r, "/proc/mounts", "/dev/mmcblk0p2 / ext4 rw 0 0\n/dev/mmcblk0p1 /boot/firmware vfat rw 0 0\nproc /proc proc rw 0 0\n")
    write(r, "/sys/bus/pci/devices/0001:01:00.0/vendor", "0x1de4\n")
    write(r, "/sys/bus/pci/devices/0001:01:00.0/device", "0x0001\n")
    write(r, "/sys/bus/pci/devices/0001:01:00.0/class", "0x020000\n")
    write(r, "/sys/bus/i2c/devices/i2c-1/name", "Synopsys DesignWare I2C adapter\n")
    write(r, "/sys/bus/i2c/devices/1-0068/name", "ds3231\n")
    write(r, "/sys/bus/usb/devices/1-1/idVendor", "046d\n")
    write(r, "/sys/bus/usb/devices/1-1/idProduct", "c52b\n")
    write(r, "/sys/bus/usb/devices/1-1/product", "USB Receiver\n")
    (r / "sys/bus/usb/devices/1-1:1.0").mkdir(parents=True)
    write(r, "/sys/class/thermal/thermal_zone0/temp", "48250\n")
    write(r, "/proc/loadavg", "0.12 0.10 0.05 1/200 1234\n")
    write(r, "/proc/uptime", "3600.5 14000.0\n")
    write(r, "/boot/firmware/config.txt", CONFIG_TXT)
    return r
