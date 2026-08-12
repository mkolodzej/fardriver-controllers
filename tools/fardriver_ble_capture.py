#!/usr/bin/env python3
"""Discover and passively capture FarDriver BLE notifications.

This tool never writes characteristic payloads. Subscribing still changes BLE
link/CCCD state and may prevent the official app from connecting concurrently.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import time
from pathlib import Path

from bleak import BleakClient, BleakScanner

SERVICE_UUID = "0000ffe0-0000-1000-8000-00805f9b34fb"
CHARACTERISTIC_PREFERENCES = (
    "0000ffec-0000-1000-8000-00805f9b34fb",
    "0000ffe1-0000-1000-8000-00805f9b34fb",
    "0000ffe2-0000-1000-8000-00805f9b34fb",
)


async def discover(seconds: float) -> list[dict[str, object]]:
    found = await BleakScanner.discover(timeout=seconds, return_adv=True)
    rows: list[dict[str, object]] = []
    for address, (device, advertisement) in found.items():
        rows.append(
            {
                "address": address,
                "name": device.name,
                "rssi": advertisement.rssi,
                "service_uuids": sorted(advertisement.service_uuids),
            }
        )
    return rows


async def capture(address: str, seconds: float, output: Path,
                  requested_characteristic: str | None) -> None:
    started = time.monotonic_ns()
    records: list[dict[str, object]] = []

    def notification(characteristic: object, data: bytearray) -> None:
        records.append(
            {
                "elapsed_us": (time.monotonic_ns() - started) // 1000,
                "direction": "rx",
                "length": len(data),
                "hex": bytes(data).hex().upper(),
                "characteristic": str(getattr(characteristic, "uuid", characteristic)),
            }
        )

    device = await BleakScanner.find_device_by_address(address, timeout=15.0)
    peer = device if device is not None else address
    async with BleakClient(
        peer, timeout=60.0, winrt={"use_cached_services": True}
    ) as client:
        services = {
            service.uuid.lower(): [
                {
                    "uuid": characteristic.uuid.lower(),
                    "properties": sorted(characteristic.properties),
                }
                for characteristic in service.characteristics
            ]
            for service in client.services
        }
        print(json.dumps({"address": address, "services": services}, indent=2))
        preferences = (
            tuple(part.strip() for part in requested_characteristic.split(","))
            if requested_characteristic else CHARACTERISTIC_PREFERENCES
        )
        characteristics = [
            candidate for uuid in preferences
            if (candidate := client.services.get_characteristic(uuid)) is not None
            and ({"notify", "indicate"} & set(candidate.properties))
        ]
        if not characteristics:
            raise RuntimeError("No supported FFE0 notification characteristic found")
        print(json.dumps({"selected_characteristics": [item.uuid for item in characteristics]}))
        for characteristic in characteristics:
            await client.start_notify(characteristic, notification)
        await asyncio.sleep(seconds)
        for characteristic in characteristics:
            await client.stop_notify(characteristic)

    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", encoding="utf-8", newline="\n") as stream:
        stream.write(json.dumps({
            "format": "fardriver-ble-capture-v2",
            "address": address,
            "requested_characteristic": requested_characteristic,
            "requested_duration_seconds": seconds,
            "payload_writes": False,
            "services": services,
            "selected_characteristics": [item.uuid for item in characteristics],
            "notification_count": len(records),
            "elapsed_us": (time.monotonic_ns() - started) // 1000,
            "completed": True,
        }) + "\n")
        for record in records:
            stream.write(json.dumps(record) + "\n")
    print(json.dumps({"notifications": len(records), "output": output.as_posix()}))


async def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scan", type=float, default=8.0)
    parser.add_argument("--address")
    parser.add_argument("--capture", type=float, default=15.0)
    parser.add_argument("--characteristic")
    parser.add_argument("--output", type=Path, default=Path("fardriver-ble-capture.jsonl"))
    args = parser.parse_args()

    if args.address:
        await capture(args.address, args.capture, args.output, args.characteristic)
        return
    print(json.dumps(await discover(args.scan), indent=2))


if __name__ == "__main__":
    asyncio.run(main())
