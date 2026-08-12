#!/usr/bin/env python3
"""Stage 1: AT QUERY probe. Operator-authorized writes, query-only commands.

Sends only interrogative AT commands (no '=' assignments), so nothing in the
module's configuration is changed. Captures every notification on all
notify-capable characteristics throughout.

Deliberately does NOT send FarDriver address writes (family C) -- no controller
parameter is touched, because no factory configuration export exists.
"""
import asyncio
import json
import sys
import time
from pathlib import Path

from bleak import BleakClient, BleakScanner

ADDRESS = "28:D4:1E:8D:29:25"
OUT = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("at_probe.jsonl")

# Query-only. No assignment ('=') forms in this stage.
QUERIES = [
    "AT",
    "AT+VERSION",
    "AT+VERSION?",
    "AT+NAME?",
    "AT+BAUD?",
    "AT+TUUID?",
    "AT+SUUID?",
    "AT+ROLE?",
    "AT+HELP",
]

WRITE_TARGETS = ["0000ffe1-0000-1000-8000-00805f9b34fb"]
records = []
started = 0


def log(stage, **kw):
    print(json.dumps({"stage": stage, **kw}), flush=True)


async def main():
    global started
    dev = None
    for i in range(5):
        dev = await BleakScanner.find_device_by_address(ADDRESS, timeout=20.0)
        if dev:
            break
        log("scan_miss", attempt=i + 1)
    if not dev:
        log("error", reason="not_found")
        return 2

    for attempt in range(1, 5):
        try:
            return await session(dev, attempt)
        except Exception as exc:  # noqa: BLE001
            log("connect_fail", attempt=attempt, error=f"{type(exc).__name__}: {exc}")
            await asyncio.sleep(3)
    return 3


async def session(dev, attempt):
    global started
    started = time.monotonic_ns()

    def on_notify(ch, data: bytearray):
        raw = bytes(data)
        rec = {
            "elapsed_us": (time.monotonic_ns() - started) // 1000,
            "direction": "rx",
            "length": len(raw),
            "hex": raw.hex().upper(),
            "ascii": raw.decode("ascii", "replace"),
            "characteristic": str(getattr(ch, "uuid", ch)),
        }
        records.append(rec)
        log("rx", **{k: rec[k] for k in ("characteristic", "length", "ascii")})

    async with BleakClient(dev, timeout=60.0) as client:
        log("connected", attempt=attempt)
        notif = []
        for svc in client.services:
            for ch in svc.characteristics:
                if {"notify", "indicate"} & set(ch.properties):
                    notif.append(ch)
        for ch in notif:
            try:
                await client.start_notify(ch, on_notify)
            except Exception as exc:  # noqa: BLE001
                log("subscribe_fail", uuid=ch.uuid, error=str(exc))
        log("subscribed", uuids=[c.uuid for c in notif])

        # Baseline: what arrives with no stimulus at all.
        await asyncio.sleep(4.0)
        base = len(records)
        log("baseline_done", notifications=base)

        for target in WRITE_TARGETS:
            ch = client.services.get_characteristic(target)
            if ch is None:
                log("write_target_missing", uuid=target)
                continue
            for q in QUERIES:
                before = len(records)
                payload = (q + "\r\n").encode()
                try:
                    resp = "write-without-response" in ch.properties
                    await client.write_gatt_char(ch, payload, response=not resp)
                    records.append({
                        "elapsed_us": (time.monotonic_ns() - started) // 1000,
                        "direction": "tx", "length": len(payload),
                        "hex": payload.hex().upper(), "ascii": q,
                        "characteristic": target,
                    })
                    log("tx", uuid=target, cmd=q)
                except Exception as exc:  # noqa: BLE001
                    log("tx_fail", uuid=target, cmd=q, error=f"{type(exc).__name__}: {exc}")
                    continue
                await asyncio.sleep(2.0)
                new = [r for r in records[before:] if r["direction"] == "rx"]
                log("reply", cmd=q, count=len(new),
                    payloads=sorted({r["ascii"] for r in new}))

        await asyncio.sleep(3.0)

        OUT.parent.mkdir(parents=True, exist_ok=True)
        with OUT.open("w", encoding="utf-8", newline="\n") as fh:
            fh.write(json.dumps({
                "format": "fardriver-ble-at-probe-v1",
                "address": ADDRESS,
                "advertised_name": dev.name,
                "stage": "at_query_only",
                "assignment_writes": False,
                "controller_parameter_writes": False,
                "queries_sent": QUERIES,
                "write_targets": WRITE_TARGETS,
                "record_count": len(records),
                "elapsed_us": (time.monotonic_ns() - started) // 1000,
                "completed": True,
            }) + "\n")
            for r in records:
                fh.write(json.dumps(r) + "\n")
        log("done", records=len(records), output=OUT.as_posix())
    return 0


sys.exit(asyncio.run(main()))
