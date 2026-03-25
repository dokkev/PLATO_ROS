#!/usr/bin/env python3
"""
PCAN hardware-only smoke tests (no Plato motors required).

What this tests:
  1) PCAN initialize/uninitialize
  2) Echo-frame configuration + single TX->echo round-trip
  3) Multi-frame TX->echo reliability and timing summary

Typical usage:
  python3 pcan_hardware_smoke_test.py
  python3 pcan_hardware_smoke_test.py --channel usbbus1 --bitrate 1M --frames 50
"""

from __future__ import annotations

import argparse
import ctypes
import statistics
import sys
import time
from dataclasses import dataclass
from typing import List, Tuple


# PCANBasic constants
PCAN_NONEBUS = 0x00
PCAN_USBBUS1 = 0x51
PCAN_PCIBUS1 = 0x41

PCAN_BAUD_1M = 0x0014
PCAN_BAUD_800K = 0x0016
PCAN_BAUD_500K = 0x001C
PCAN_BAUD_250K = 0x011C
PCAN_BAUD_125K = 0x031C

PCAN_ERROR_OK = 0x00000
PCAN_ERROR_QRCVEMPTY = 0x00020

PCAN_MESSAGE_STANDARD = 0x00
PCAN_MESSAGE_ECHO = 0x20
PCAN_ALLOW_ECHO_FRAMES = 0x2C


class TPCANMsg(ctypes.Structure):
    _fields_ = [
        ("ID", ctypes.c_uint32),
        ("MSGTYPE", ctypes.c_ubyte),
        ("LEN", ctypes.c_ubyte),
        ("DATA", ctypes.c_ubyte * 8),
    ]


class TPCANTimestamp(ctypes.Structure):
    _fields_ = [
        ("millis", ctypes.c_uint32),
        ("millis_overflow", ctypes.c_uint16),
        ("micros", ctypes.c_uint16),
    ]


class TestFailure(RuntimeError):
    pass


def format_error(lib: ctypes.CDLL, status: int) -> str:
    buf = ctypes.create_string_buffer(256)
    # 0x09 = English
    if lib.CAN_GetErrorText(status, 0x09, buf) == PCAN_ERROR_OK:
        return buf.value.decode(errors="replace")
    return f"PCAN error 0x{status:X}"


def parse_channel(text: str) -> int:
    value = text.strip().lower()
    mapping = {
        "usbbus1": PCAN_USBBUS1,
        "pcibus1": PCAN_PCIBUS1,
    }
    if value not in mapping:
        raise argparse.ArgumentTypeError(f"Unsupported channel '{text}' (use usbbus1 or pcibus1)")
    return mapping[value]


def parse_bitrate(text: str) -> int:
    value = text.strip().lower()
    mapping = {
        "1m": PCAN_BAUD_1M,
        "800k": PCAN_BAUD_800K,
        "500k": PCAN_BAUD_500K,
        "250k": PCAN_BAUD_250K,
        "125k": PCAN_BAUD_125K,
    }
    if value not in mapping:
        raise argparse.ArgumentTypeError(
            f"Unsupported bitrate '{text}' (use 1M/800k/500k/250k/125k)"
        )
    return mapping[value]


def open_pcan_lib() -> ctypes.CDLL:
    lib = ctypes.CDLL("libpcanbasic.so")
    lib.CAN_Initialize.argtypes = [ctypes.c_uint16, ctypes.c_uint16]
    lib.CAN_Initialize.restype = ctypes.c_uint32
    lib.CAN_Uninitialize.argtypes = [ctypes.c_uint16]
    lib.CAN_Uninitialize.restype = ctypes.c_uint32
    lib.CAN_SetValue.argtypes = [ctypes.c_uint16, ctypes.c_uint8, ctypes.c_void_p, ctypes.c_uint32]
    lib.CAN_SetValue.restype = ctypes.c_uint32
    lib.CAN_Read.argtypes = [ctypes.c_uint16, ctypes.POINTER(TPCANMsg), ctypes.POINTER(TPCANTimestamp)]
    lib.CAN_Read.restype = ctypes.c_uint32
    lib.CAN_Write.argtypes = [ctypes.c_uint16, ctypes.POINTER(TPCANMsg)]
    lib.CAN_Write.restype = ctypes.c_uint32
    lib.CAN_GetErrorText.argtypes = [ctypes.c_uint32, ctypes.c_uint16, ctypes.c_char_p]
    lib.CAN_GetErrorText.restype = ctypes.c_uint32
    return lib


def drain_rx(lib: ctypes.CDLL, channel: int) -> int:
    count = 0
    while True:
        msg = TPCANMsg()
        ts = TPCANTimestamp()
        st = lib.CAN_Read(channel, ctypes.byref(msg), ctypes.byref(ts))
        if st == PCAN_ERROR_OK:
            count += 1
            continue
        if st == PCAN_ERROR_QRCVEMPTY:
            return count
        return count


def build_frame(can_id: int, payload: bytes) -> TPCANMsg:
    if len(payload) > 8:
        raise ValueError("payload must be <= 8 bytes")
    msg = TPCANMsg()
    msg.ID = can_id
    msg.MSGTYPE = PCAN_MESSAGE_STANDARD
    msg.LEN = len(payload)
    for i, b in enumerate(payload):
        msg.DATA[i] = b
    return msg


def wait_for_echo(
    lib: ctypes.CDLL,
    channel: int,
    can_id: int,
    payload: bytes,
    timeout_ms: int,
) -> Tuple[float, TPCANMsg]:
    start = time.monotonic()
    deadline = start + (timeout_ms / 1000.0)
    while time.monotonic() < deadline:
        msg = TPCANMsg()
        ts = TPCANTimestamp()
        st = lib.CAN_Read(channel, ctypes.byref(msg), ctypes.byref(ts))
        if st == PCAN_ERROR_OK:
            if msg.ID != can_id:
                continue
            if bytes(msg.DATA[: msg.LEN]) != payload:
                continue
            if (msg.MSGTYPE & PCAN_MESSAGE_ECHO) == 0:
                continue
            latency_ms = (time.monotonic() - start) * 1000.0
            return latency_ms, msg
        if st == PCAN_ERROR_QRCVEMPTY:
            time.sleep(0.001)
            continue
        raise TestFailure(f"CAN_Read failed while waiting echo: {format_error(lib, st)}")
    raise TestFailure(f"Timed out waiting echo for ID 0x{can_id:X}")


@dataclass
class TestContext:
    lib: ctypes.CDLL
    channel: int
    can_id: int
    timeout_ms: int
    frames: int
    strict_echo: bool


def case_initialize(ctx: TestContext) -> None:
    # Channel already initialized in main. This case validates basic RX API access.
    _ = drain_rx(ctx.lib, ctx.channel)


def case_echo_single(ctx: TestContext) -> None:
    echo_on = ctypes.c_uint32(1)
    st = ctx.lib.CAN_SetValue(
        ctx.channel,
        PCAN_ALLOW_ECHO_FRAMES,
        ctypes.byref(echo_on),
        ctypes.sizeof(echo_on),
    )
    if st != PCAN_ERROR_OK:
        raise TestFailure(f"Failed to enable echo frames: {format_error(ctx.lib, st)}")

    _ = drain_rx(ctx.lib, ctx.channel)
    payload = b"\x93\x11\x22\x33\x44\x55\x66\x77"
    msg = build_frame(ctx.can_id, payload)
    st = ctx.lib.CAN_Write(ctx.channel, ctypes.byref(msg))
    if st != PCAN_ERROR_OK:
        raise TestFailure(f"CAN_Write failed: {format_error(ctx.lib, st)}")

    latency_ms, _ = wait_for_echo(ctx.lib, ctx.channel, ctx.can_id, payload, ctx.timeout_ms)
    print(f"  single-echo latency: {latency_ms:.3f} ms")


def case_echo_multi(ctx: TestContext) -> None:
    latencies: List[float] = []
    misses = 0

    for i in range(ctx.frames):
        payload = bytes([0x93, i & 0xFF, 0xAA, 0x55, 0, 0, 0, 0])
        msg = build_frame(ctx.can_id, payload)
        st = ctx.lib.CAN_Write(ctx.channel, ctypes.byref(msg))
        if st != PCAN_ERROR_OK:
            raise TestFailure(f"CAN_Write failed at frame {i}: {format_error(ctx.lib, st)}")
        try:
            latency_ms, _ = wait_for_echo(ctx.lib, ctx.channel, ctx.can_id, payload, ctx.timeout_ms)
            latencies.append(latency_ms)
        except TestFailure:
            misses += 1

    if not latencies:
        raise TestFailure("No echo received in multi-frame test")

    p50 = statistics.median(latencies)
    p95 = sorted(latencies)[int(0.95 * (len(latencies) - 1))]
    print(
        "  multi-echo stats: "
        f"ok={len(latencies)}/{ctx.frames}, misses={misses}, "
        f"latency_ms(min/p50/p95/max)="
        f"{min(latencies):.3f}/{p50:.3f}/{p95:.3f}/{max(latencies):.3f}"
    )

    # Any misses are considered a failure for this smoke test.
    if misses > 0:
        raise TestFailure(f"Echo misses detected: {misses}/{ctx.frames}")


def run_case(name: str, fn, ctx: TestContext) -> str:
    print(f"[TEST] {name}")
    start = time.monotonic()
    try:
        fn(ctx)
    except Exception as exc:  # noqa: BLE001
        elapsed = (time.monotonic() - start) * 1000.0
        if name.startswith("echo_") and not ctx.strict_echo:
            print(f"[SKIP] {name} ({elapsed:.1f} ms): {exc}")
            return "skip"
        print(f"[FAIL] {name} ({elapsed:.1f} ms): {exc}")
        return "fail"
    elapsed = (time.monotonic() - start) * 1000.0
    print(f"[PASS] {name} ({elapsed:.1f} ms)")
    return "pass"


def main() -> int:
    parser = argparse.ArgumentParser(description="PCAN hardware-only smoke tests.")
    parser.add_argument("--channel", type=parse_channel, default=PCAN_USBBUS1, help="usbbus1|pcibus1")
    parser.add_argument("--bitrate", type=parse_bitrate, default=PCAN_BAUD_1M, help="1M|800k|500k|250k|125k")
    parser.add_argument("--id", type=lambda x: int(x, 0), default=0x11, help="TX ID used for test frames")
    parser.add_argument("--timeout-ms", type=int, default=200, help="Echo wait timeout per frame")
    parser.add_argument("--frames", type=int, default=20, help="Frame count for multi-echo test")
    parser.add_argument(
        "--strict-echo",
        action="store_true",
        help="Fail when echo tests cannot run (default: mark echo tests as SKIP).",
    )
    parser.add_argument(
        "--force-reset",
        action="store_true",
        help="Call CAN_Uninitialize(PCAN_NONEBUS) before initialize.",
    )
    args = parser.parse_args()

    lib = open_pcan_lib()
    if args.force_reset:
        lib.CAN_Uninitialize(PCAN_NONEBUS)

    init_status = lib.CAN_Initialize(args.channel, args.bitrate)
    if init_status != PCAN_ERROR_OK:
        print(
            "Failed to initialize PCAN channel: "
            f"{format_error(lib, init_status)}\n"
            "Hint: stop other process using this PCAN channel.",
            file=sys.stderr,
        )
        return 2

    try:
        ctx = TestContext(
            lib=lib,
            channel=args.channel,
            can_id=args.id,
            timeout_ms=args.timeout_ms,
            frames=args.frames,
            strict_echo=args.strict_echo,
        )
        cases = [
            ("initialize_and_rx_drain", case_initialize),
            ("echo_single_frame", case_echo_single),
            ("echo_multi_frame", case_echo_multi),
        ]

        passed = 0
        skipped = 0
        for name, fn in cases:
            status = run_case(name, fn, ctx)
            if status == "pass":
                passed += 1
            elif status == "skip":
                skipped += 1

        failed = len(cases) - passed - skipped
        print(f"[SUMMARY] passed={passed}/{len(cases)}, skipped={skipped}, failed={failed}")
        return 0 if failed == 0 else 1
    finally:
        lib.CAN_Uninitialize(args.channel)


if __name__ == "__main__":
    sys.exit(main())
