#!/usr/bin/env python3
"""Collect and validate QMI8658 v0.1.2 byte-order hardware evidence."""

from __future__ import annotations

import argparse
import datetime as dt
import json
import math
import platform
import queue
import statistics
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Any

MARKERS = {
    "QMI_EVIDENCE_BEGIN",
    "QMI_EVIDENCE_META",
    "QMI_EVIDENCE_DIRECT",
    "QMI_EVIDENCE_FIFO_META",
    "QMI_EVIDENCE_FIFO",
    "QMI_EVIDENCE_ERROR",
    "QMI_EVIDENCE_END",
}
VARIANTS = ("qmi8658a", "qmi8658c")
TRANSPORTS = ("i2c", "spi")
BYTE_ORDERS = ("le", "be")
ACCEL_LSB_PER_G = 4096.0  # QA firmware uses the Config default: +/-8 g.
GYRO_LSB_PER_DPS = 64.0  # QA firmware uses the Config default: +/-512 dps.
TEMP_LSB_PER_C = 256.0
DIRECT_RAW_BYTES = 17
FIFO_RAW_BYTES = 12


class EvidenceError(RuntimeError):
    """Evidence collection or validation failed."""


def utc_now() -> str:
    return dt.datetime.now(dt.timezone.utc).replace(microsecond=0).isoformat()


def run_command(
    command: list[str], cwd: Path, *, check: bool = True
) -> subprocess.CompletedProcess[str]:
    print("+", subprocess.list2cmdline(command), file=sys.stderr)
    return subprocess.run(
        command,
        cwd=cwd,
        check=check,
        text=True,
        encoding="utf-8",
        errors="replace",
    )


def git_value(repo_root: Path, *args: str) -> str:
    result = subprocess.run(
        ["git", *args],
        cwd=repo_root,
        check=False,
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
    )
    return result.stdout.strip() if result.returncode == 0 else "unknown"


def source_changes(repo_root: Path, output_dir: Path) -> list[str]:
    result = subprocess.run(
        ["git", "status", "--porcelain", "--untracked-files=normal"],
        cwd=repo_root,
        check=False,
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
    )
    if result.returncode != 0:
        return ["git-status-unavailable"]
    ignored_prefixes = ["target/", "tmp/"]
    try:
        relative_output = output_dir.relative_to(repo_root).as_posix().rstrip("/")
        ignored_prefixes.append(f"{relative_output}/")
    except ValueError:
        pass
    changes: list[str] = []
    for line in result.stdout.splitlines():
        path = line[3:].strip()
        if " -> " in path:
            path = path.rsplit(" -> ", 1)[1]
        normalized = path.replace("\\", "/")
        if any(normalized.startswith(prefix) for prefix in ignored_prefixes):
            continue
        if "/__pycache__/" in f"/{normalized}/" or normalized.endswith((".pyc", ".pyo")):
            continue
        changes.append(line)
    return changes


def build_firmware(app_dir: Path, features: list[str]) -> Path:
    run_command(
        [
            "cargo",
            "+esp",
            "build",
            "--release",
            "--no-default-features",
            "--features",
            ",".join(features),
        ],
        app_dir,
    )
    repo_root = app_dir.parents[1]
    image = (
        repo_root
        / "target"
        / "xtensa-esp32s3-none-elf"
        / "release"
        / "imu-hw-tests"
    )
    if not image.is_file():
        raise EvidenceError(f"firmware image was not produced: {image}")
    return image


def capture_live_log(image: Path, port: str, timeout_seconds: float) -> str:
    command = [
        "espflash",
        "flash",
        "--chip",
        "esp32s3",
        "--port",
        port,
        "--monitor",
        "--log-format",
        "defmt",
        str(image),
    ]
    print("+", subprocess.list2cmdline(command), file=sys.stderr)
    process = subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        encoding="utf-8",
        errors="replace",
        bufsize=1,
    )
    assert process.stdout is not None
    lines: queue.Queue[str | None] = queue.Queue()

    def read_output() -> None:
        for line in process.stdout:
            lines.put(line)
        lines.put(None)

    reader = threading.Thread(target=read_output, daemon=True)
    reader.start()
    deadline = time.monotonic() + timeout_seconds
    captured: list[str] = []
    saw_end = False
    try:
        while time.monotonic() < deadline:
            try:
                line = lines.get(timeout=0.25)
            except queue.Empty:
                if process.poll() is not None:
                    break
                continue
            if line is None:
                break
            captured.append(line)
            print(line, end="")
            if "QMI_EVIDENCE_END" in line:
                saw_end = True
                break
    finally:
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5)
    if not saw_end:
        raise EvidenceError(
            f"collector did not receive QMI_EVIDENCE_END within {timeout_seconds:g}s"
        )
    return "".join(captured)


def marker_payload(line: str) -> tuple[str, dict[str, str]] | None:
    positions = [(line.find(marker), marker) for marker in MARKERS]
    found = [(position, marker) for position, marker in positions if position >= 0]
    if not found:
        return None
    position, marker = min(found, key=lambda item: (item[0], -len(item[1])))
    payload = line[position + len(marker) :].strip()
    values: dict[str, str] = {}
    for token in payload.split():
        if "=" not in token:
            continue
        key, value = token.split("=", 1)
        values[key] = value
    return marker, values


def parse_decimal(values: dict[str, str], key: str) -> int:
    try:
        return int(values[key], 10)
    except (KeyError, ValueError) as error:
        raise EvidenceError(f"invalid or missing decimal field {key!r}") from error


def parse_hex(values: dict[str, str], key: str) -> int:
    try:
        return int(values[key], 16)
    except (KeyError, ValueError) as error:
        raise EvidenceError(f"invalid or missing hexadecimal field {key!r}") from error


def parse_sample(values: dict[str, str], raw_bytes: int) -> dict[str, Any]:
    raw = values.get("raw", "")
    if len(raw) != raw_bytes * 2:
        raise EvidenceError(
            f"raw sample has {len(raw) // 2} bytes; expected {raw_bytes}"
        )
    try:
        bytes.fromhex(raw)
    except ValueError as error:
        raise EvidenceError("raw sample is not valid hexadecimal") from error
    sample: dict[str, Any] = {
        "index": parse_decimal(values, "index"),
        "raw": raw.lower(),
    }
    for key in ("timestamp", "temp", "ax", "ay", "az", "gx", "gy", "gz"):
        if key in values:
            sample[key] = parse_decimal(values, key)
    return sample


def parse_log(log_text: str) -> dict[str, Any]:
    record: dict[str, Any] = {
        "kind": "qmi8658-be-evidence-run",
        "schema": 1,
        "run": {},
        "identity": {},
        "direct_samples": [],
        "fifo": {},
        "fifo_samples": [],
        "firmware_errors": [],
        "end_result": "missing",
    }
    for line in log_text.splitlines():
        parsed = marker_payload(line)
        if parsed is None:
            continue
        marker, values = parsed
        if marker == "QMI_EVIDENCE_BEGIN":
            record["schema"] = parse_decimal(values, "schema")
            record["run"] = {
                "variant": values.get("variant"),
                "transport": values.get("transport"),
                "byte_order": "be" if parse_decimal(values, "be") else "le",
                "orientation": values.get("orientation"),
                "expected_direct_samples": parse_decimal(values, "direct_samples"),
                "fifo_buffer_bytes": parse_decimal(values, "fifo_buffer_bytes"),
            }
        elif marker == "QMI_EVIDENCE_META":
            record["identity"] = {
                "who_am_i": parse_hex(values, "who_am_i"),
                "revision_id": parse_hex(values, "revision_id"),
                "ctrl1": parse_hex(values, "ctrl1"),
            }
        elif marker == "QMI_EVIDENCE_DIRECT":
            record["direct_samples"].append(parse_sample(values, DIRECT_RAW_BYTES))
        elif marker == "QMI_EVIDENCE_FIFO_META":
            record["fifo"] = {
                "bytes_read": parse_decimal(values, "bytes_read"),
                "sample_count_bytes": parse_decimal(values, "sample_count_bytes"),
                "overflow": bool(parse_decimal(values, "overflow")),
            }
        elif marker == "QMI_EVIDENCE_FIFO":
            record["fifo_samples"].append(parse_sample(values, FIFO_RAW_BYTES))
        elif marker == "QMI_EVIDENCE_ERROR":
            record["firmware_errors"].append(values)
        elif marker == "QMI_EVIDENCE_END":
            record["end_result"] = values.get("result", "missing")
    return record


def decode_i16(raw: bytes, big_endian: bool) -> int:
    return int.from_bytes(raw, byteorder="big" if big_endian else "little", signed=True)


def decoded_direct(raw_hex: str, big_endian: bool) -> dict[str, int]:
    raw = bytes.fromhex(raw_hex)
    return {
        "timestamp": int.from_bytes(raw[0:3], "little"),
        "temp": decode_i16(raw[3:5], big_endian),
        "ax": decode_i16(raw[5:7], big_endian),
        "ay": decode_i16(raw[7:9], big_endian),
        "az": decode_i16(raw[9:11], big_endian),
        "gx": decode_i16(raw[11:13], big_endian),
        "gy": decode_i16(raw[13:15], big_endian),
        "gz": decode_i16(raw[15:17], big_endian),
    }


def decoded_fifo(raw_hex: str, big_endian: bool) -> dict[str, int]:
    raw = bytes.fromhex(raw_hex)
    return {
        "ax": decode_i16(raw[0:2], big_endian),
        "ay": decode_i16(raw[2:4], big_endian),
        "az": decode_i16(raw[4:6], big_endian),
        "gx": decode_i16(raw[6:8], big_endian),
        "gy": decode_i16(raw[8:10], big_endian),
        "gz": decode_i16(raw[10:12], big_endian),
    }


def mean(samples: list[dict[str, Any]], key: str) -> float:
    return statistics.fmean(float(sample[key]) for sample in samples)


def physical_metrics(samples: list[dict[str, Any]], include_temperature: bool) -> dict[str, Any]:
    accel = {axis: mean(samples, f"a{axis}") / ACCEL_LSB_PER_G for axis in "xyz"}
    gyro = {axis: mean(samples, f"g{axis}") / GYRO_LSB_PER_DPS for axis in "xyz"}
    metrics: dict[str, Any] = {
        "sample_count": len(samples),
        "mean_accel_g": accel,
        "mean_accel_magnitude_g": math.sqrt(sum(value * value for value in accel.values())),
        "mean_gyro_dps": gyro,
    }
    if include_temperature:
        metrics["mean_temperature_c"] = mean(samples, "temp") / TEMP_LSB_PER_C
    return metrics


def add_check(checks: list[dict[str, Any]], name: str, passed: bool, detail: str) -> None:
    checks.append({"name": name, "passed": bool(passed), "detail": detail})


def validate_run(
    record: dict[str, Any],
    *,
    expected_variant: str | None = None,
    expected_transport: str | None = None,
    expected_byte_order: str | None = None,
    ambient_celsius: float | None = None,
) -> dict[str, Any]:
    checks: list[dict[str, Any]] = []
    run = record.get("run", {})
    identity = record.get("identity", {})
    direct = record.get("direct_samples", [])
    fifo = record.get("fifo_samples", [])
    byte_order = run.get("byte_order")
    big_endian = byte_order == "be"

    if expected_variant is not None:
        add_check(
            checks,
            "requested variant",
            run.get("variant") == expected_variant,
            f"firmware={run.get('variant')} requested={expected_variant}",
        )
    if expected_transport is not None:
        add_check(
            checks,
            "requested transport",
            run.get("transport") == expected_transport,
            f"firmware={run.get('transport')} requested={expected_transport}",
        )
    if expected_byte_order is not None:
        add_check(
            checks,
            "requested byte order",
            byte_order == expected_byte_order,
            f"firmware={byte_order} requested={expected_byte_order}",
        )
    add_check(
        checks,
        "firmware completion",
        record.get("end_result") == "pass",
        f"result={record.get('end_result')}",
    )
    add_check(
        checks,
        "WHO_AM_I",
        identity.get("who_am_i") == 0x05,
        f"readback=0x{identity.get('who_am_i', -1):02x}",
    )
    ctrl1 = identity.get("ctrl1")
    add_check(
        checks,
        "CTRL1.BE readback",
        isinstance(ctrl1, int) and bool(ctrl1 & 0x20) == big_endian,
        f"CTRL1=0x{ctrl1 if isinstance(ctrl1, int) else -1:02x} requested={byte_order}",
    )
    if run.get("variant") == "qmi8658c":
        add_check(
            checks,
            "QMI8658C reserved CTRL1 bits",
            isinstance(ctrl1, int) and ctrl1 & 0x18 == 0,
            f"CTRL1=0x{ctrl1 if isinstance(ctrl1, int) else -1:02x}",
        )
    expected_count = int(run.get("expected_direct_samples") or 0)
    add_check(
        checks,
        "direct sample count",
        expected_count > 0 and len(direct) == expected_count,
        f"captured={len(direct)} expected={expected_count}",
    )
    add_check(checks, "FIFO sample count", len(fifo) > 0, f"captured={len(fifo)}")

    raw_matches = True
    try:
        for sample in direct:
            decoded = decoded_direct(sample["raw"], big_endian)
            raw_matches &= all(sample.get(key) == value for key, value in decoded.items())
        for sample in fifo:
            decoded = decoded_fifo(sample["raw"], big_endian)
            raw_matches &= all(sample.get(key) == value for key, value in decoded.items())
    except (KeyError, ValueError):
        raw_matches = False
    add_check(
        checks,
        "raw/decoded agreement",
        raw_matches,
        "firmware fields must exactly decode from the captured raw bytes",
    )

    metrics: dict[str, Any] = {}
    if direct:
        direct_metrics = physical_metrics(direct, include_temperature=True)
        metrics["direct"] = direct_metrics
        accel = direct_metrics["mean_accel_g"]
        magnitude = direct_metrics["mean_accel_magnitude_g"]
        orientation_ok = (
            abs(accel["x"]) <= 0.35
            and abs(accel["y"]) <= 0.35
            and 0.65 <= accel["z"] <= 1.35
            and 0.75 <= magnitude <= 1.25
        )
        add_check(
            checks,
            "static +Z 1 g orientation",
            orientation_ok,
            f"mean_g=({accel['x']:.3f},{accel['y']:.3f},{accel['z']:.3f}) magnitude={magnitude:.3f}",
        )
        temperature = direct_metrics["mean_temperature_c"]
        temperature_ok = -40.0 <= temperature <= 85.0
        if ambient_celsius is not None:
            temperature_ok &= abs(temperature - ambient_celsius) <= 20.0
        detail = f"mean={temperature:.2f}C"
        if ambient_celsius is not None:
            detail += f" ambient={ambient_celsius:.2f}C"
        add_check(checks, "temperature plausibility", temperature_ok, detail)
        gyro = direct_metrics["mean_gyro_dps"]
        gyro_ok = all(abs(gyro[axis]) <= 20.0 for axis in "xyz")
        add_check(
            checks,
            "stationary gyro plausibility",
            gyro_ok,
            f"mean_dps=({gyro['x']:.2f},{gyro['y']:.2f},{gyro['z']:.2f})",
        )
    if fifo:
        fifo_metrics = physical_metrics(fifo, include_temperature=False)
        metrics["fifo"] = fifo_metrics
        accel = fifo_metrics["mean_accel_g"]
        magnitude = fifo_metrics["mean_accel_magnitude_g"]
        orientation_ok = (
            abs(accel["x"]) <= 0.35
            and abs(accel["y"]) <= 0.35
            and 0.65 <= accel["z"] <= 1.35
            and 0.75 <= magnitude <= 1.25
        )
        add_check(
            checks,
            "FIFO static +Z 1 g orientation",
            orientation_ok,
            f"mean_g=({accel['x']:.3f},{accel['y']:.3f},{accel['z']:.3f}) magnitude={magnitude:.3f}",
        )
    if direct and fifo:
        direct_accel = metrics["direct"]["mean_accel_g"]
        fifo_accel = metrics["fifo"]["mean_accel_g"]
        max_delta = max(abs(direct_accel[axis] - fifo_accel[axis]) for axis in "xyz")
        add_check(
            checks,
            "direct/FIFO consistency",
            max_delta <= 0.35,
            f"maximum mean-axis delta={max_delta:.3f}g",
        )

    validation = {
        "status": "pass" if checks and all(check["passed"] for check in checks) else "fail",
        "checks": checks,
        "metrics": metrics,
    }
    record["validation"] = validation
    return validation


def markdown_run_summary(record: dict[str, Any]) -> str:
    host = record.get("host", {})
    run = record.get("run", {})
    identity = record.get("identity", {})
    validation = record.get("validation", {})
    lines = [
        "# QMI8658 BE Hardware Evidence Run",
        "",
        f"- Status: **{validation.get('status', 'fail').upper()}**",
        f"- Collected: {host.get('collected_at_utc', 'unknown')}",
        f"- Operator: {host.get('operator', 'unknown')}",
        f"- Board: {host.get('board_id', 'unknown')}",
        f"- Sensor: {host.get('sensor_id', 'unknown')}",
        f"- Variant: `{run.get('variant', 'unknown')}`",
        f"- Transport: `{run.get('transport', 'unknown')}`",
        f"- Byte order: `{run.get('byte_order', 'unknown')}`",
        f"- Orientation: `{run.get('orientation', 'unknown')}`",
        f"- Source commit: `{host.get('source_commit', 'unknown')}`",
        (
            f"- Identity: WHO_AM_I=`0x{identity.get('who_am_i', 0):02x}`, "
            f"REVISION_ID=`0x{identity.get('revision_id', 0):02x}`, "
            f"CTRL1=`0x{identity.get('ctrl1', 0):02x}`"
        ),
        "",
        "## Automated checks",
        "",
        "| Check | Result | Detail |",
        "|---|---|---|",
    ]
    for check in validation.get("checks", []):
        result = "PASS" if check["passed"] else "FAIL"
        detail = str(check["detail"]).replace("|", "\\|")
        lines.append(f"| {check['name']} | {result} | {detail} |")
    lines.extend(
        [
            "",
            "The JSON file is authoritative and contains all raw direct/FIFO bytes.",
            "A passing run does not resolve BE-01 until the complete eight-row matrix passes.",
            "",
        ]
    )
    return "\n".join(lines)


def safe_component(value: str) -> str:
    cleaned = "".join(character if character.isalnum() or character in "-_" else "-" for character in value)
    return cleaned.strip("-") or "unknown"


def collect(args: argparse.Namespace) -> int:
    script = Path(__file__).resolve()
    app_dir = script.parents[1]
    repo_root = script.parents[3]
    output_dir = Path(args.output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    if args.input_log:
        log_text = Path(args.input_log).read_text(encoding="utf-8", errors="replace")
    else:
        if not args.port:
            raise EvidenceError("--port is required for live collection")
        features = [args.variant, f"transport-{args.transport}"]
        if args.byte_order == "be":
            features.append("big-endian")
        image = build_firmware(app_dir, features)
        log_text = capture_live_log(image, args.port, args.timeout)

    record = parse_log(log_text)
    source_commit = git_value(repo_root, "rev-parse", "HEAD")
    changes = source_changes(repo_root, output_dir)
    record["host"] = {
        "collected_at_utc": utc_now(),
        "operator": args.operator,
        "board_id": args.board_id,
        "sensor_id": args.sensor_id,
        "ambient_celsius": args.ambient_celsius,
        "notes": args.notes,
        "port": args.port,
        "source_commit": source_commit,
        "source_dirty": bool(changes),
        "source_changes": changes,
        "python": sys.version.split()[0],
        "host_platform": platform.platform(),
        "collector": str(script.relative_to(repo_root)),
    }
    validate_run(
        record,
        expected_variant=args.variant,
        expected_transport=args.transport,
        expected_byte_order=args.byte_order,
        ambient_celsius=args.ambient_celsius,
    )

    timestamp = (
        record["host"]["collected_at_utc"]
        .replace("+00:00", "Z")
        .replace(":", "")
        .replace("-", "")
    )
    stem = "-".join(
        (
            args.variant,
            args.transport,
            args.byte_order,
            safe_component(args.sensor_id),
            timestamp,
        )
    )
    log_path = output_dir / f"{stem}.log"
    json_path = output_dir / f"{stem}.json"
    markdown_path = output_dir / f"{stem}.md"
    log_path.write_text(log_text, encoding="utf-8")
    json_path.write_text(json.dumps(record, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    markdown_path.write_text(markdown_run_summary(record), encoding="utf-8")
    print(f"raw log: {log_path}")
    print(f"evidence: {json_path}")
    print(f"summary: {markdown_path}")
    return 0 if record["validation"]["status"] == "pass" else 2


def latest_matrix_runs(input_dir: Path) -> tuple[dict[tuple[str, str, str], dict[str, Any]], list[str]]:
    selected: dict[tuple[str, str, str], dict[str, Any]] = {}
    problems: list[str] = []
    for path in sorted(input_dir.glob("*.json")):
        try:
            record = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as error:
            problems.append(f"{path.name}: unreadable JSON ({error})")
            continue
        if record.get("kind") != "qmi8658-be-evidence-run":
            continue
        run = record.get("run", {})
        key = (run.get("variant"), run.get("transport"), run.get("byte_order"))
        if key[0] not in VARIANTS or key[1] not in TRANSPORTS or key[2] not in BYTE_ORDERS:
            problems.append(f"{path.name}: invalid matrix key {key}")
            continue
        record["_path"] = str(path)
        previous = selected.get(key)
        previous_time = previous.get("host", {}).get("collected_at_utc", "") if previous else ""
        current_time = record.get("host", {}).get("collected_at_utc", "")
        if previous is None or current_time >= previous_time:
            selected[key] = record
    return selected, problems


def metric_delta(
    first: dict[str, Any], second: dict[str, Any], source: str, metric: str
) -> float:
    first_value = first["validation"]["metrics"][source][metric]
    second_value = second["validation"]["metrics"][source][metric]
    if isinstance(first_value, dict):
        return max(abs(first_value[axis] - second_value[axis]) for axis in first_value)
    return abs(float(first_value) - float(second_value))


def validate_matrix(
    records: dict[tuple[str, str, str], dict[str, Any]],
    problems: list[str],
    expected_source_commit: str | None = None,
) -> dict[str, Any]:
    expected = {
        (variant, transport, byte_order)
        for variant in VARIANTS
        for transport in TRANSPORTS
        for byte_order in BYTE_ORDERS
    }
    missing = sorted(expected - records.keys())
    checks: list[dict[str, Any]] = []
    add_check(checks, "complete eight-row matrix", not missing, f"missing={missing}")
    add_check(checks, "input files readable", not problems, "; ".join(problems) or "ok")

    failed_runs = sorted(
        key
        for key, record in records.items()
        if record.get("validation", {}).get("status") != "pass"
    )
    add_check(checks, "all individual runs pass", not failed_runs, f"failed={failed_runs}")
    dirty_runs = sorted(
        key
        for key, record in records.items()
        if record.get("host", {}).get("source_dirty") is not False
    )
    add_check(
        checks,
        "all runs use clean source trees",
        not dirty_runs,
        f"dirty_or_unknown={dirty_runs}",
    )
    source_commits = {
        record.get("host", {}).get("source_commit")
        for record in records.values()
        if record.get("host", {}).get("source_commit")
    }
    add_check(
        checks,
        "source commit consistency",
        len(source_commits) == 1 and len(records) == 8,
        f"commits={sorted(source_commits)}",
    )
    if expected_source_commit is not None:
        add_check(
            checks,
            "expected source commit",
            source_commits == {expected_source_commit},
            f"recorded={sorted(source_commits)} expected={expected_source_commit}",
        )

    for variant in VARIANTS:
        variant_records = [record for key, record in records.items() if key[0] == variant]
        sensor_ids = {
            record.get("host", {}).get("sensor_id")
            for record in variant_records
            if record.get("host", {}).get("sensor_id")
        }
        revisions = {
            record.get("identity", {}).get("revision_id")
            for record in variant_records
            if isinstance(record.get("identity", {}).get("revision_id"), int)
        }
        add_check(
            checks,
            f"{variant} sensor identity consistency",
            len(sensor_ids) == 1 and len(variant_records) == 4,
            f"sensor_ids={sorted(sensor_ids)}",
        )
        add_check(
            checks,
            f"{variant} revision consistency",
            len(revisions) == 1 and len(variant_records) == 4,
            f"revision_ids={[f'0x{value:02x}' for value in sorted(revisions)]}",
        )

    for variant in VARIANTS:
        for transport in TRANSPORTS:
            le = records.get((variant, transport, "le"))
            be = records.get((variant, transport, "be"))
            if not le or not be:
                continue
            try:
                accel_delta = metric_delta(le, be, "direct", "mean_accel_g")
                gyro_delta = metric_delta(le, be, "direct", "mean_gyro_dps")
                temp_delta = metric_delta(le, be, "direct", "mean_temperature_c")
                fifo_delta = metric_delta(le, be, "fifo", "mean_accel_g")
            except (KeyError, TypeError, ValueError):
                add_check(
                    checks,
                    f"{variant}/{transport} BE direct consistency",
                    False,
                    "required physical metrics unavailable",
                )
                add_check(
                    checks,
                    f"{variant}/{transport} BE FIFO consistency",
                    False,
                    "required physical metrics unavailable",
                )
                continue
            add_check(
                checks,
                f"{variant}/{transport} BE direct consistency",
                accel_delta <= 0.25 and gyro_delta <= 10.0 and temp_delta <= 5.0,
                f"accel={accel_delta:.3f}g gyro={gyro_delta:.2f}dps temp={temp_delta:.2f}C",
            )
            add_check(
                checks,
                f"{variant}/{transport} BE FIFO consistency",
                fifo_delta <= 0.25,
                f"accel={fifo_delta:.3f}g",
            )

    for variant in VARIANTS:
        for byte_order in BYTE_ORDERS:
            i2c = records.get((variant, "i2c", byte_order))
            spi = records.get((variant, "spi", byte_order))
            if not i2c or not spi:
                continue
            try:
                accel_delta = metric_delta(i2c, spi, "direct", "mean_accel_g")
                gyro_delta = metric_delta(i2c, spi, "direct", "mean_gyro_dps")
                temp_delta = metric_delta(i2c, spi, "direct", "mean_temperature_c")
                fifo_delta = metric_delta(i2c, spi, "fifo", "mean_accel_g")
            except (KeyError, TypeError, ValueError):
                add_check(
                    checks,
                    f"{variant}/{byte_order} transport direct consistency",
                    False,
                    "required physical metrics unavailable",
                )
                add_check(
                    checks,
                    f"{variant}/{byte_order} transport FIFO consistency",
                    False,
                    "required physical metrics unavailable",
                )
                continue
            add_check(
                checks,
                f"{variant}/{byte_order} transport direct consistency",
                accel_delta <= 0.25 and gyro_delta <= 10.0 and temp_delta <= 5.0,
                f"accel={accel_delta:.3f}g gyro={gyro_delta:.2f}dps temp={temp_delta:.2f}C",
            )
            add_check(
                checks,
                f"{variant}/{byte_order} transport FIFO consistency",
                fifo_delta <= 0.25,
                f"accel={fifo_delta:.3f}g",
            )

    return {
        "kind": "qmi8658-be-evidence-matrix",
        "schema": 1,
        "generated_at_utc": utc_now(),
        "status": "pass" if checks and all(check["passed"] for check in checks) else "blocked",
        "checks": checks,
        "runs": {
            "/".join(key): {
                "path": record["_path"],
                "collected_at_utc": record.get("host", {}).get("collected_at_utc"),
                "sensor_id": record.get("host", {}).get("sensor_id"),
                "source_commit": record.get("host", {}).get("source_commit"),
                "revision_id": record.get("identity", {}).get("revision_id"),
                "validation": record.get("validation", {}).get("status"),
            }
            for key, record in sorted(records.items())
        },
    }


def markdown_matrix_summary(matrix: dict[str, Any]) -> str:
    lines = [
        "# QMI8658 BE-01 Hardware Matrix",
        "",
        f"- Status: **{matrix['status'].upper()}**",
        f"- Generated: {matrix['generated_at_utc']}",
        "",
        "## Selected runs",
        "",
        "| Variant | Transport | Byte order | Sensor | Revision | Source | Run status |",
        "|---|---|---|---|---|---|---|",
    ]
    for key, run in matrix["runs"].items():
        variant, transport, byte_order = key.split("/")
        revision = run.get("revision_id")
        revision_text = f"`0x{revision:02x}`" if isinstance(revision, int) else "missing"
        lines.append(
            f"| {variant} | {transport} | {byte_order} | "
            f"{run.get('sensor_id') or 'missing'} | {revision_text} | "
            f"`{run.get('source_commit') or 'missing'}` | {run.get('validation')} |"
        )
    lines.extend(
        [
            "",
            "## Matrix checks",
            "",
            "| Check | Result | Detail |",
            "|---|---|---|",
        ]
    )
    for check in matrix["checks"]:
        result = "PASS" if check["passed"] else "BLOCKED"
        detail = str(check["detail"]).replace("|", "\\|")
        lines.append(f"| {check['name']} | {result} | {detail} |")
    lines.extend(
        [
            "",
            "BE-01 may be marked Done only when this report says PASS and the run artifacts are reviewed.",
            "",
        ]
    )
    return "\n".join(lines)


def matrix(args: argparse.Namespace) -> int:
    input_dir = Path(args.input_dir).resolve()
    output_dir = Path(args.output_dir or input_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    records, problems = latest_matrix_runs(input_dir)
    result = validate_matrix(records, problems, args.expected_source_commit)
    json_path = output_dir / "BE-01-MATRIX.json"
    markdown_path = output_dir / "BE-01-MATRIX.md"
    json_path.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    markdown_path.write_text(markdown_matrix_summary(result), encoding="utf-8")
    print(f"matrix evidence: {json_path}")
    print(f"matrix summary: {markdown_path}")
    print(f"status: {result['status']}")
    return 0 if result["status"] == "pass" else 2


def parser() -> argparse.ArgumentParser:
    root = argparse.ArgumentParser(description=__doc__)
    subcommands = root.add_subparsers(dest="command", required=True)

    collect_parser = subcommands.add_parser(
        "collect", help="build/flash firmware or parse a captured serial log"
    )
    collect_parser.add_argument("--variant", choices=VARIANTS, required=True)
    collect_parser.add_argument("--transport", choices=TRANSPORTS, required=True)
    collect_parser.add_argument("--byte-order", choices=BYTE_ORDERS, required=True)
    collect_parser.add_argument("--port", help="ESP32-S3 serial port for live collection")
    collect_parser.add_argument("--operator", required=True)
    collect_parser.add_argument("--board-id", required=True)
    collect_parser.add_argument("--sensor-id", required=True)
    collect_parser.add_argument("--ambient-celsius", type=float)
    collect_parser.add_argument("--notes", default="")
    collect_parser.add_argument(
        "--timeout", type=float, default=120.0, help="serial evidence timeout in seconds"
    )
    collect_parser.add_argument(
        "--input-log", help="parse an existing log instead of building/flashing"
    )
    collect_parser.add_argument(
        "--output-dir",
        default="hardware-evidence/v0.1.2",
        help="artifact directory (default: hardware-evidence/v0.1.2)",
    )
    collect_parser.set_defaults(handler=collect)

    matrix_parser = subcommands.add_parser(
        "matrix", help="validate the latest complete A/C x I2C/SPI x LE/BE matrix"
    )
    matrix_parser.add_argument(
        "--input-dir",
        default="hardware-evidence/v0.1.2",
        help="directory containing run JSON files",
    )
    matrix_parser.add_argument("--output-dir")
    matrix_parser.add_argument(
        "--expected-source-commit",
        help="require every selected run to identify this exact source commit",
    )
    matrix_parser.set_defaults(handler=matrix)
    return root


def main() -> int:
    args = parser().parse_args()
    try:
        return int(args.handler(args))
    except (EvidenceError, OSError, subprocess.CalledProcessError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
