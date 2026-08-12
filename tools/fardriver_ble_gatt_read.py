#!/usr/bin/env python3
"""Direct GATT READs — genuinely untried on this adapter.

Every prior session used notify-subscription or writes. A GATT read is a
different operation: it returns the characteristic's stored value. FFE1 carries
the 'read' property and has never been read.

Also reads 1800/2A00 (device name), 2A01 (appearance) and 2A04 (conn params).
2A00 may name the module, which is the open low-confidence identity question
(deep-research finding 18).

Read-only: no write of any kind.
"""
import asyncio
import json
import sys

from bleak import BleakClient, BleakScanner

ADDRESS = "28:D4:1E:8D:29:25"

READABLE = [
    ("2A00 device name", "00002a00-0000-1000-8000-00805f9b34fb"),
    ("2A01 appearance", "00002a01-0000-1000-8000-00805f9b34fb"),
    ("2A04 conn params", "00002a04-0000-1000-8000-00805f9b34fb"),
]
# Try these even though enumeration reports no 'read' property — a device may
# still answer, and a failure is itself information.
SPECULATIVE = [
    ("FFE2", "0000ffe2-0000-1000-8000-00805f9b34fb"),
    ("FFE3", "0000ffe3-0000-1000-8000-00805f9b34fb"),
    ("FF11", "0000ff11-0000-1000-8000-00805f9b34fb"),
    ("FF12", "0000ff12-0000-1000-8000-00805f9b34fb"),
    ("FFE1 transparent LAST", "0000ffe1-0000-1000-8000-00805f9b34fb"),
]


def log(**kw):
    print(json.dumps(kw), flush=True)


async def main():
    dev = None
    for i in range(6):
        dev = await BleakScanner.find_device_by_address(ADDRESS, timeout=20.0)
        if dev:
            break
        log(stage="scan_miss", attempt=i + 1)
    if not dev:
        log(stage="error", reason="not_found")
        return 2

    for attempt in range(1, 8):
        try:
            async with BleakClient(dev, timeout=60.0) as client:
                log(stage="connected", attempt=attempt)
                for label, uuid in READABLE + SPECULATIVE:
                    ch = client.services.get_characteristic(uuid)
                    if ch is None:
                        log(stage="read", target=label, result="absent")
                        continue
                    try:
                        raw = bytes(await client.read_gatt_char(ch))
                        log(stage="read", target=label,
                            props=sorted(ch.properties),
                            length=len(raw), hex=raw.hex().upper(),
                            ascii=raw.decode("ascii", "replace"))
                    except Exception as exc:  # noqa: BLE001
                        log(stage="read", target=label,
                            props=sorted(ch.properties),
                            error=f"{type(exc).__name__}: {exc}")
                log(stage="done")
                return 0
        except Exception as exc:  # noqa: BLE001
            log(stage="connect_fail", attempt=attempt,
                error=f"{type(exc).__name__}: {exc}")
            await asyncio.sleep(5)
    log(stage="error", reason="connect_exhausted")
    return 3


sys.exit(asyncio.run(main()))
