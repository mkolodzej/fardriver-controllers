#!/usr/bin/env python3
"""Does a GATT READ of FFE1 change device state, or only drop the link?

Tests the operator's hypothesis directly, and also settles a loose end I left:
in the AT-probe session the receive rate went 2.53/s (pre-TX) -> 6.67/s (post-TX)
and I attributed that to connection settling WITHOUT proving it.

Design:
  PHASE 1  connect, subscribe, capture 20 s untouched  -> baseline cadence,
           measured in two halves so settling is visible as a within-phase trend
  PHASE 2  read FFE1 (expected to drop the link), record what happens
  PHASE 3  reconnect, subscribe, capture 20 s untouched -> post-read cadence

If phase 3 == phase 1, the read is link-local and does NOT alter device state.
If phase 3 differs (stops, slows, new payloads), the read changed something.

Read-only: one GATT read. No writes of any kind.
"""
import asyncio
import json
import statistics
import sys
import time
from pathlib import Path

from bleak import BleakClient, BleakScanner

ADDRESS = "28:D4:1E:8D:29:25"
FFE1 = "0000ffe1-0000-1000-8000-00805f9b34fb"
OUT = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("read_effect.jsonl")
WINDOW = 20.0

records = []


def log(**kw):
    print(json.dumps(kw), flush=True)


async def find():
    for i in range(8):
        d = await BleakScanner.find_device_by_address(ADDRESS, timeout=20.0)
        if d:
            return d
        log(stage="scan_miss", attempt=i + 1)
    return None


def stats(rows, tag):
    if len(rows) < 3:
        log(stage="stats", phase=tag, rx=len(rows), note="too few to characterise")
        return None
    ts = [r["t_us"] for r in rows]
    gaps = sorted((ts[i + 1] - ts[i]) / 1000.0 for i in range(len(ts) - 1)
                  if ts[i + 1] > ts[i])
    span = (ts[-1] - ts[0]) / 1e6
    half = len(rows) // 2
    def rate(sub):
        if len(sub) < 2:
            return 0.0
        s = (sub[-1]["t_us"] - sub[0]["t_us"]) / 1e6
        return len(sub) / s if s > 0 else 0.0
    out = {
        "phase": tag, "rx": len(rows), "span_s": round(span, 1),
        "rate_per_s": round(len(rows) / span, 2) if span else 0,
        "median_gap_ms": round(statistics.median(gaps), 1),
        "first_half_rate": round(rate(rows[:half]), 2),
        "second_half_rate": round(rate(rows[half:]), 2),
        "distinct_payloads": sorted({r["hex"] for r in rows})[:3],
        "non_at": sum(1 for r in rows if set(bytes.fromhex(r["hex"])) - set(b"AT\r\n")),
    }
    log(stage="stats", **out)
    return out


async def capture(dev, phase, seconds):
    got = []
    t0 = time.monotonic_ns()

    def on_notify(ch, data: bytearray):
        raw = bytes(data)
        rec = {"phase": phase, "t_us": (time.monotonic_ns() - t0) // 1000,
               "len": len(raw), "hex": raw.hex().upper(),
               "char": str(getattr(ch, "uuid", ch))}
        got.append(rec)
        records.append(rec)

    for attempt in range(1, 8):
        try:
            async with BleakClient(dev, timeout=60.0) as client:
                log(stage="connected", phase=phase, attempt=attempt)
                for svc in client.services:
                    for ch in svc.characteristics:
                        if {"notify", "indicate"} & set(ch.properties):
                            try:
                                await client.start_notify(ch, on_notify)
                            except Exception:  # noqa: BLE001, S110
                                pass
                await asyncio.sleep(seconds)
                return got, client
        except Exception as exc:  # noqa: BLE001
            log(stage="connect_fail", phase=phase, attempt=attempt,
                error=f"{type(exc).__name__}: {exc}")
            await asyncio.sleep(5)
    return got, None


async def main():
    dev = await find()
    if not dev:
        log(stage="error", reason="not_found")
        return 2

    # PHASE 1 + 2 share one connection: capture, then read, then observe.
    got1 = []
    t0 = time.monotonic_ns()

    def on_notify(ch, data: bytearray):
        raw = bytes(data)
        rec = {"phase": "1_baseline" if not read_done["v"] else "2_after_read",
               "t_us": (time.monotonic_ns() - t0) // 1000,
               "len": len(raw), "hex": raw.hex().upper(),
               "char": str(getattr(ch, "uuid", ch))}
        got1.append(rec)
        records.append(rec)

    read_done = {"v": False}
    for attempt in range(1, 8):
        try:
            async with BleakClient(dev, timeout=60.0) as client:
                log(stage="connected", phase="1_baseline", attempt=attempt)
                for svc in client.services:
                    for ch in svc.characteristics:
                        if {"notify", "indicate"} & set(ch.properties):
                            try:
                                await client.start_notify(ch, on_notify)
                            except Exception:  # noqa: BLE001, S110
                                pass
                await asyncio.sleep(WINDOW)
                base = [r for r in got1 if r["phase"] == "1_baseline"]
                stats(base, "1_baseline")

                log(stage="READ_FFE1", note="expected to drop the link")
                read_done["v"] = True
                try:
                    raw = bytes(await client.read_gatt_char(FFE1))
                    log(stage="read_ok", hex=raw.hex().upper(), length=len(raw))
                except Exception as exc:  # noqa: BLE001
                    log(stage="read_failed", error=f"{type(exc).__name__}: {exc}")
                # Did notifications survive the read?
                await asyncio.sleep(6.0)
                after = [r for r in got1 if r["phase"] == "2_after_read"]
                log(stage="post_read_in_session", rx=len(after),
                    still_connected=client.is_connected)
                break
        except Exception as exc:  # noqa: BLE001
            log(stage="connect_fail", phase="1_baseline", attempt=attempt,
                error=f"{type(exc).__name__}: {exc}")
            await asyncio.sleep(5)

    # PHASE 3: fresh connection, untouched capture.
    log(stage="phase3", note="reconnecting for clean post-read baseline")
    await asyncio.sleep(4.0)
    dev2 = await find() or dev
    got3, _ = await capture(dev2, "3_post_read", WINDOW)
    s3 = stats(got3, "3_post_read")

    OUT.parent.mkdir(parents=True, exist_ok=True)
    with OUT.open("w", encoding="utf-8", newline="\n") as fh:
        fh.write(json.dumps({
            "format": "fardriver-ble-read-effect-v1",
            "address": ADDRESS, "window_s": WINDOW,
            "writes": 0, "gatt_reads": 1,
            "record_count": len(records), "completed": True,
        }) + "\n")
        for r in records:
            fh.write(json.dumps(r) + "\n")
    log(stage="done", records=len(records), output=OUT.as_posix())
    return 0


sys.exit(asyncio.run(main()))
