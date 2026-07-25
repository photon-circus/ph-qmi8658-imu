#!/usr/bin/env python3
"""Tests for the hardware evidence parser and matrix validator."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import collect_evidence as evidence


def i16(value: int, big_endian: bool) -> bytes:
    return value.to_bytes(2, byteorder="big" if big_endian else "little", signed=True)


def sample_log(
    variant: str = "qmi8658c",
    transport: str = "i2c",
    byte_order: str = "be",
) -> str:
    big_endian = byte_order == "be"
    ctrl1 = 0x60 if big_endian else 0x40
    revision = 0x7C if variant == "qmi8658a" else 0x68
    lines = [
        (
            "QMI_EVIDENCE_BEGIN schema=1 "
            f"variant={variant} transport={transport} be={int(big_endian)} "
            "orientation=z-up direct_samples=16 fifo_buffer_bytes=192"
        ),
        (
            "QMI_EVIDENCE_META "
            f"who_am_i=05 revision_id={revision:02x} ctrl1={ctrl1:02x}"
        ),
    ]
    direct_raw = (
        b"\x01\x02\x03"
        + i16(25 * 256, big_endian)
        + i16(0, big_endian)
        + i16(0, big_endian)
        + i16(4096, big_endian)
        + i16(0, big_endian)
        + i16(0, big_endian)
        + i16(0, big_endian)
    )
    for index in range(16):
        lines.append(
            f"QMI_EVIDENCE_DIRECT index={index} raw={direct_raw.hex()} "
            "timestamp=197121 temp=6400 ax=0 ay=0 az=4096 gx=0 gy=0 gz=0"
        )
    fifo_raw = (
        i16(0, big_endian)
        + i16(0, big_endian)
        + i16(4096, big_endian)
        + i16(0, big_endian)
        + i16(0, big_endian)
        + i16(0, big_endian)
    )
    lines.append("QMI_EVIDENCE_FIFO_META bytes_read=24 sample_count_bytes=24 overflow=0")
    for index in range(2):
        lines.append(
            f"QMI_EVIDENCE_FIFO index={index} raw={fifo_raw.hex()} "
            "ax=0 ay=0 az=4096 gx=0 gy=0 gz=0"
        )
    lines.append("QMI_EVIDENCE_END result=pass")
    return "\n".join(lines)


def passing_record(variant: str, transport: str, byte_order: str) -> dict:
    record = evidence.parse_log(sample_log(variant, transport, byte_order))
    record["host"] = {
        "collected_at_utc": f"2026-07-25T00:00:0{int(byte_order == 'be')}+00:00",
        "sensor_id": f"{variant}-sensor-1",
        "source_commit": "0123456789abcdef",
        "source_dirty": False,
    }
    evidence.validate_run(
        record,
        expected_variant=variant,
        expected_transport=transport,
        expected_byte_order=byte_order,
        ambient_celsius=25.0,
    )
    record["_path"] = f"{variant}-{transport}-{byte_order}.json"
    return record


class EvidenceTests(unittest.TestCase):
    def test_parse_and_validate_passing_run(self) -> None:
        record = passing_record("qmi8658c", "i2c", "be")

        self.assertEqual(record["identity"]["who_am_i"], 0x05)
        self.assertEqual(record["identity"]["revision_id"], 0x68)
        self.assertEqual(len(record["direct_samples"]), 16)
        self.assertEqual(len(record["fifo_samples"]), 2)
        self.assertEqual(record["validation"]["status"], "pass")

    def test_raw_decode_mismatch_fails(self) -> None:
        log = sample_log().replace("az=4096", "az=4095", 1)
        record = evidence.parse_log(log)
        validation = evidence.validate_run(record)

        self.assertEqual(validation["status"], "fail")
        raw_check = next(
            check for check in validation["checks"] if check["name"] == "raw/decoded agreement"
        )
        self.assertFalse(raw_check["passed"])

    def test_complete_matrix_passes(self) -> None:
        records = {
            (variant, transport, byte_order): passing_record(
                variant, transport, byte_order
            )
            for variant in evidence.VARIANTS
            for transport in evidence.TRANSPORTS
            for byte_order in evidence.BYTE_ORDERS
        }

        matrix = evidence.validate_matrix(records, [], "0123456789abcdef")

        self.assertEqual(matrix["status"], "pass")
        self.assertEqual(len(matrix["runs"]), 8)

    def test_incomplete_matrix_remains_blocked(self) -> None:
        records = {
            ("qmi8658c", "i2c", "be"): passing_record("qmi8658c", "i2c", "be")
        }

        matrix = evidence.validate_matrix(records, [])

        self.assertEqual(matrix["status"], "blocked")

    def test_failed_run_without_metrics_blocks_instead_of_crashing(self) -> None:
        records = {
            (variant, transport, byte_order): passing_record(
                variant, transport, byte_order
            )
            for variant in evidence.VARIANTS
            for transport in evidence.TRANSPORTS
            for byte_order in evidence.BYTE_ORDERS
        }
        failed = records[("qmi8658a", "spi", "le")]
        failed["validation"] = {"status": "fail", "checks": [], "metrics": {}}

        matrix = evidence.validate_matrix(records, [])

        self.assertEqual(matrix["status"], "blocked")


if __name__ == "__main__":
    unittest.main()
