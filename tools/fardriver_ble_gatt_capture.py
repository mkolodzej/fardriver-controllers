#!/usr/bin/env python3
"""Connect, enumerate real GATT, and passively capture every notify-capable
characteristic. Writes NO characteristic payloads.

Retries connection because the link is weak (RSSI ~-82..-90).
"""
import asyncio
import json
import sys
import time
from pathlib import Path

from bleak import BleakClient, BleakScanner

ADDRESS = "28:D4:1E:8D:29:25"
CAPTURE_SECONDS = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
OUT = Path(sys.argv[2]) if len(sys.argv) > 2 else Path("ble_full.jsonl")
ATTEMPTS = 5


async def find():
    for i in range(ATTEMPTS):
        dev = await BleakScanner.find_device_by_address(ADDRESS, timeout=20.0)
        if dev is not None:
            print(json.dumps({"stage": "found", "attempt": i + 1, "name": dev.name}), flush=True)
            return dev
        print(json.dumps({"stage": "scan_miss", "attempt": i + 1}), flush=True)
    return None


async def run():
    dev = await find()
    if dev is None:
        print(json.dumps({"error": "not_found"}))
        return 2

    last_err = None
    for attempt in range(1, ATTEMPTS + 1):
        try:
            return await session(dev, attempt)
        except Exception as exc:  # noqa: BLE001 - report any transport failure
            last_err = f"{type(exc).__name__}: {exc}"
            print(json.dumps({"stage": "connect_fail", "attempt": attempt,
                              "error": last_err}), flush=True)
            await asyncio.sleep(3.0)
    print(json.dumps({"error": "connect_exhausted", "last": last_err}))
    return 3


async def session(dev, attempt):
    started = time.monotonic_ns()
    records = []

    def on_notify(ch, data: bytearray):
        records.append({
            "elapsed_us": (time.monotonic_ns() - started) // 1000,
            "direction": "rx",
            "length": len(data),
            "hex": bytes(data).hex().upper(),
            "characteristic": str(getattr(ch, "uuid", ch)),
        })

    async with BleakClient(dev, timeout=60.0) as client:
        print(json.dumps({"stage": "connected", "attempt": attempt}), flush=True)
        services = []
        notifiable = []
        for svc in client.services:
            row = {"uuid": svc.uuid, "characteristics": []}
            for ch in svc.characteristics:
                row["characteristics"].append({
                    "uuid": ch.uuid,
                    "handle": ch.handle,
                    "properties": sorted(ch.properties),
                })
                if {"notify", "indicate"} & set(ch.properties):
                    notifiable.append(ch)
            services.append(row)
        print(json.dumps({"stage": "gatt", "services": services}, indent=2), flush=True)
        print(json.dumps({"stage": "subscribing",
                          "characteristics": [c.uuid for c in notifiable]}), flush=True)

        subscribed = []
        for ch in notifiable:
            try:
                await client.start_notify(ch, on_notify)
                subscribed.append(ch)
            except Exception as exc:  # noqa: BLE001
                print(json.dumps({"stage": "subscribe_fail", "uuid": ch.uuid,
                                  "error": f"{type(exc).__name__}: {exc}"}), flush=True)

        print(json.dumps({"stage": "capturing", "seconds": CAPTURE_SECONDS,
                          "subscribed": [c.uuid for c in subscribed]}), flush=True)
        await asyncio.sleep(CAPTURE_SECONDS)

        for ch in subscribed:
            try:
                await client.stop_notify(ch)
            except Exception:  # noqa: BLE001, S110 - best-effort teardown
                pass

        elapsed = (time.monotonic_ns() - started) // 1000
        OUT.parent.mkdir(parents=True, exist_ok=True)
        with OUT.open("w", encoding="utf-8", newline="\n") as fh:
            fh.write(json.dumps({
                "format": "fardriver-ble-capture-v2",
                "address": ADDRESS,
                "advertised_name": dev.name,
                "requested_duration_seconds": CAPTURE_SECONDS,
                "payload_writes": False,
                "services": services,
                "selected_characteristics": [c.uuid for c in subscribed],
                "notification_count": len(records),
                "elapsed_us": elapsed,
                "completed": True,
            }) + "\n")
            for r in records:
                fh.write(json.dumps(r) + "\n")

        by_ch = {}
        for r in records:
            by_ch[r["characteristic"]] = by_ch.get(r["characteristic"], 0) + 1
        print(json.dumps({"stage": "done", "notifications": len(records),
                          "by_characteristic": by_ch,
                          "output": OUT.as_posix()}, indent=2), flush=True)
    return 0


sys.exit(asyncio.run(run()))
