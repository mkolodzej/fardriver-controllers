#!/usr/bin/env python3
"""Write escalation ladder. Operator-authorized (2026-08-12, reaffirmed).

Stages, cheapest/most-reversible first. Aborts early the moment real telemetry
appears (any non-AT payload, especially a 0xAA-led 16-byte frame).

  A  AT 'OK' reply on EVERY writable characteristic, not just FFE1.
     Prior tests only wrote FFE1; FFE2/FFE3/FF11/FF12 were never tried.
  B  Timing-tight reply: answer within milliseconds of each AT rather than on a
     fixed schedule, in case the controller has a short response window.
  C  FarDriver legacy session commands (family D) on each writable char. These
     are what the PC app actually sends to start status streaming. NOT parameter
     writes -- no configuration address is touched, so no restore point needed.
     Golden bytes taken from tests/protocol_vectors.cpp (CRC-verified).

  D  THE CONFIGURABLE CHANNEL (operator-directed). AT+TUUID=FFEC repoints the
     module's transparent-transmission UUID at the characteristic the official
     app expects, then AT+RESET/AT+REBOOT to apply. PERSISTENT -- not undone by
     a power cycle. Restore with AT+TUUID=FFE1 (the value the current GATT
     layout implies). Still module config, NOT a FarDriver parameter write.

NOT sent at any stage: FarDriver address writes (family C / parameter changes).
The controller's factory configuration was never exported (register #23).
"""
import asyncio
import json
import sys
import time
from pathlib import Path

from bleak import BleakClient, BleakScanner

ADDRESS = "28:D4:1E:8D:29:25"
OUT = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("escalate.jsonl")

FFE1 = "0000ffe1-0000-1000-8000-00805f9b34fb"
FFE2 = "0000ffe2-0000-1000-8000-00805f9b34fb"
FFE3 = "0000ffe3-0000-1000-8000-00805f9b34fb"
FF11 = "0000ff11-0000-1000-8000-00805f9b34fb"
FF12 = "0000ff12-0000-1000-8000-00805f9b34fb"
WRITABLE = [FFE1, FFE2, FFE3, FF11, FF12]

# CRC-verified in tests/protocol_vectors.cpp.
FD_OPEN = bytes([0xAA, 0x13, 0xEC, 0x07, 0x01, 0xF1, 0xA2, 0x5D])
FD_KEEPALIVE = bytes([0xAA, 0x13, 0xEC, 0x07, 0x5F, 0x5F, 0x6E, 0x91])
FD_PCPOLL = bytes([0xAA, 0x05, 0xFA, 0x01, 0x5F, 0x5F, 0x68, 0x97])

records = []
started = 0
hits = []


def log(stage, **kw):
    print(json.dumps({"stage": stage, **kw}), flush=True)


def is_at(raw: bytes) -> bool:
    return len(raw) > 0 and not (set(raw) - set(b"AT\r\n"))


async def write(client, uuid, payload, label):
    ch = client.services.get_characteristic(uuid)
    if ch is None:
        log("skip", uuid=uuid[4:8], reason="absent")
        return False
    if not ({"write", "write-without-response"} & set(ch.properties)):
        log("skip", uuid=uuid[4:8], reason="not writable")
        return False
    no_resp = "write-without-response" in ch.properties
    try:
        await client.write_gatt_char(ch, payload, response=not no_resp)
    except Exception as exc:  # noqa: BLE001
        log("tx_fail", uuid=uuid[4:8], label=label, error=f"{type(exc).__name__}: {exc}")
        return False
    records.append({
        "elapsed_us": (time.monotonic_ns() - started) // 1000,
        "direction": "tx", "length": len(payload),
        "hex": payload.hex().upper(), "label": label, "characteristic": uuid,
    })
    log("tx", uuid=uuid[4:8], label=label, hex=payload.hex().upper())
    return True


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
    for attempt in range(1, 8):
        try:
            return await session(dev, attempt)
        except Exception as exc:  # noqa: BLE001
            log("connect_fail", attempt=attempt, error=f"{type(exc).__name__}: {exc}")
            await asyncio.sleep(5)
    log("error", reason="connect_exhausted")
    return 3


async def session(dev, attempt):
    global started
    started = time.monotonic_ns()
    tight = {"on": False, "client": None, "ch": None}

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
        if not is_at(raw):
            hits.append(rec)
            log("*** NON_AT ***", hex=rec["hex"][:100], length=rec["length"],
                uuid=rec["characteristic"][4:8], ascii=rec["ascii"][:40])
        if tight["on"]:
            asyncio.create_task(_tight_reply())

    async def _tight_reply():
        try:
            await tight["client"].write_gatt_char(tight["ch"], b"OK\r\n", response=False)
        except Exception:  # noqa: BLE001, S110
            pass

    async def watch(seconds, tag):
        before = len(hits)
        await asyncio.sleep(seconds)
        rx = [r for r in records if r["direction"] == "rx"]
        log("watch", tag=tag, total_rx=len(rx), new_non_at=len(hits) - before)
        return len(hits) > before

    async with BleakClient(dev, timeout=60.0) as client:
        log("connected", attempt=attempt)
        for svc in client.services:
            for ch in svc.characteristics:
                if {"notify", "indicate"} & set(ch.properties):
                    try:
                        await client.start_notify(ch, on_notify)
                    except Exception:  # noqa: BLE001, S110
                        pass
        await watch(4.0, "baseline")

        # ---- Stage A: OK reply on every writable characteristic ----
        log("STAGE", name="A", desc="AT OK on each writable characteristic")
        for uuid in WRITABLE:
            if await write(client, uuid, b"OK\r\n", "OK"):
                if await watch(3.5, f"A:{uuid[4:8]}"):
                    log("BREAKTHROUGH", stage="A", uuid=uuid[4:8])
                    return await finish(client)

        # ---- Stage B: timing-tight reply on FFE1 ----
        log("STAGE", name="B", desc="immediate OK on each received AT (FFE1)")
        tight["client"] = client
        tight["ch"] = client.services.get_characteristic(FFE1)
        if tight["ch"] is not None:
            tight["on"] = True
            got = await watch(12.0, "B:tight")
            tight["on"] = False
            await asyncio.sleep(1.0)
            if got:
                log("BREAKTHROUGH", stage="B")
                return await finish(client)

        # ---- Stage C: FarDriver legacy session commands ----
        log("STAGE", name="C", desc="FarDriver family-D session commands")
        for label, pkt in (("Open", FD_OPEN), ("KeepAlive", FD_KEEPALIVE),
                           ("PcPoll", FD_PCPOLL)):
            for uuid in WRITABLE:
                if await write(client, uuid, pkt, label):
                    if await watch(3.0, f"C:{label}:{uuid[4:8]}"):
                        log("BREAKTHROUGH", stage="C", label=label, uuid=uuid[4:8])
                        return await finish(client)

        # Open then repeated KeepAlive on FFE1, mimicking the PC app's session.
        log("STAGE", name="C2", desc="Open + repeated KeepAlive on FFE1")
        await write(client, FFE1, FD_OPEN, "Open")
        for i in range(6):
            await asyncio.sleep(2.0)
            await write(client, FFE1, FD_KEEPALIVE, f"KeepAlive{i+1}")
        if await watch(6.0, "C2:settle"):
            log("BREAKTHROUGH", stage="C2")
            return await finish(client)

        # ---- Stage D: the CONFIGURABLE CHANNEL (operator-directed) ----
        # PERSISTENT. Repoints the module's transparent-transmission UUID at the
        # characteristic the official app expects. Restore with AT+TUUID=FFE1,
        # which is the value the current GATT layout implies. Not undone by a
        # power cycle. Still module config -- NOT a FarDriver parameter write.
        log("STAGE", name="D", desc="AT+TUUID=FFEC assignment (PERSISTENT)")
        log("RESTORE_HINT", command="AT+TUUID=FFE1",
            note="current GATT exposes FFE1/FFE2/FFE3, so FFE1 is the implied original")
        for uuid in (FFE1, FFE2, FFE3):
            if await write(client, uuid, b"AT+TUUID=FFEC\r\n", "TUUID=FFEC"):
                if await watch(4.0, f"D:{uuid[4:8]}"):
                    log("BREAKTHROUGH", stage="D", uuid=uuid[4:8])
                    return await finish(client)
        # Some modules need the assignment committed and/or a reset to apply.
        for cmd in (b"AT+RESET\r\n", b"AT+REBOOT\r\n"):
            await write(client, FFE1, cmd, cmd.decode().strip())
            await watch(4.0, f"D:{cmd.decode().strip()}")

        return await finish(client)


async def finish(client):
    rx = [r for r in records if r["direction"] == "rx"]
    tx = [r for r in records if r["direction"] == "tx"]
    OUT.parent.mkdir(parents=True, exist_ok=True)
    with OUT.open("w", encoding="utf-8", newline="\n") as fh:
        fh.write(json.dumps({
            "format": "fardriver-ble-escalation-v1",
            "address": ADDRESS,
            "stages": ["A: AT OK all writable chars", "B: tight-timed OK",
                       "C: FarDriver family-D session commands", "C2: Open+KeepAlive", "D: AT+TUUID=FFEC assignment (persistent)"],
            "parameter_writes": False,
            "assignment_writes": True,
            "tx_count": len(tx), "rx_count": len(rx),
            "non_at_count": len(hits),
            "elapsed_us": (time.monotonic_ns() - started) // 1000,
            "completed": True,
        }) + "\n")
        for r in records:
            fh.write(json.dumps(r) + "\n")
    log("done", tx=len(tx), rx=len(rx), non_at=len(hits),
        distinct=sorted({r["hex"][:32] for r in rx})[:4], output=OUT.as_posix())
    return 0


sys.exit(asyncio.run(main()))
