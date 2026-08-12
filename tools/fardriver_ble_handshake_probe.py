#!/usr/bin/env python3
"""Stage 2: answer the periodic AT handshake.

HYPOTHESIS: the ~150 ms `AT\\r\\n` stream is the CONTROLLER probing its Bluetooth
module over UART. The module is in transparent mode, so it relays the
controller's AT bytes to BLE notify instead of consuming them, and the
controller retries forever because nothing ever answers. Upstream notes support
the direction: controller commands 0x06/0x0A/0x10 emit `AT+BAUD=19200`,
`AT+NAME=...`, `AT+TUUID=FFEC` -- i.e. the CONTROLLER issues AT commands at a
module.

TEST: write plausible module-style acknowledgements back and watch for the
stream to change character -- ideally to 0xAA-led 16-byte status frames.

Operator-authorized writes. Still sends NO FarDriver address write (family C),
so no controller parameter is modified; there is no factory-config restore point.
"""
import asyncio
import json
import sys
import time
from pathlib import Path

from bleak import BleakClient, BleakScanner

ADDRESS = "28:D4:1E:8D:29:25"
OUT = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("handshake.jsonl")
FFE1 = "0000ffe1-0000-1000-8000-00805f9b34fb"

# Replies a module would send to a host's AT probe, in escalating order.
REPLIES = [
    b"OK\r\n",
    b"OK\r\n",
    b"+OK\r\n",
    b"AT+OK\r\n",
    b"OK",
]

records = []
started = 0


def log(stage, **kw):
    print(json.dumps({"stage": stage, **kw}), flush=True)


def summarize(rs):
    out = {}
    for r in rs:
        out[r["ascii"][:24]] = out.get(r["ascii"][:24], 0) + 1
    return out


async def main():
    dev = None
    for i in range(6):
        dev = await BleakScanner.find_device_by_address(ADDRESS, timeout=20.0)
        if dev:
            break
        log("scan_miss", attempt=i + 1)
    if not dev:
        log("error", reason="not_found")
        return 2
    for attempt in range(1, 15):
        try:
            return await session(dev, attempt)
        except Exception as exc:  # noqa: BLE001
            log("connect_fail", attempt=attempt, error=f"{type(exc).__name__}: {exc}")
            await asyncio.sleep(6)
    return 3


async def session(dev, attempt):
    global started
    started = time.monotonic_ns()
    interesting = []

    def on_notify(ch, data: bytearray):
        raw = bytes(data)
        rec = {
            "elapsed_us": (time.monotonic_ns() - started) // 1000,
            "direction": "rx", "length": len(raw),
            "hex": raw.hex().upper(),
            "ascii": raw.decode("ascii", "replace"),
            "characteristic": str(getattr(ch, "uuid", ch)),
        }
        records.append(rec)
        if 0xAA in raw or raw.strip(b"AT\r\n"):
            interesting.append(rec)
            log("NON_AT_PAYLOAD", hex=rec["hex"][:80], length=rec["length"],
                uuid=rec["characteristic"].split("-")[0][-4:])

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
            except Exception:  # noqa: BLE001, S110
                pass
        ch = client.services.get_characteristic(FFE1)
        no_resp = "write-without-response" in ch.properties

        await asyncio.sleep(4.0)
        base = len(records)
        log("baseline", rx=base, payloads=summarize(records))

        for i, reply in enumerate(REPLIES, 1):
            before = len(records)
            try:
                await client.write_gatt_char(ch, reply, response=not no_resp)
                records.append({
                    "elapsed_us": (time.monotonic_ns() - started) // 1000,
                    "direction": "tx", "length": len(reply),
                    "hex": reply.hex().upper(),
                    "ascii": reply.decode("ascii", "replace"),
                    "characteristic": FFE1,
                })
                log("tx", n=i, payload=reply.decode("ascii", "replace").strip())
            except Exception as exc:  # noqa: BLE001
                log("tx_fail", n=i, error=f"{type(exc).__name__}: {exc}")
                continue
            await asyncio.sleep(4.0)
            new = [r for r in records[before:] if r["direction"] == "rx"]
            log("window", n=i, rx=len(new), payloads=summarize(new))

        log("settle")
        await asyncio.sleep(8.0)

        rx = [r for r in records if r["direction"] == "rx"]
        OUT.parent.mkdir(parents=True, exist_ok=True)
        with OUT.open("w", encoding="utf-8", newline="\n") as fh:
            fh.write(json.dumps({
                "format": "fardriver-ble-handshake-v1",
                "address": ADDRESS, "advertised_name": dev.name,
                "hypothesis": "periodic AT is the controller probing its BLE module",
                "replies_sent": [r.decode('ascii', 'replace') for r in REPLIES],
                "controller_parameter_writes": False,
                "rx_count": len(rx),
                "non_at_payloads": len(interesting),
                "elapsed_us": (time.monotonic_ns() - started) // 1000,
                "completed": True,
            }) + "\n")
            for r in records:
                fh.write(json.dumps(r) + "\n")
        log("done", rx=len(rx), non_at=len(interesting),
            distinct=sorted({r["ascii"][:24] for r in rx}), output=OUT.as_posix())
    return 0


sys.exit(asyncio.run(main()))
