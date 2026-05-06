"""
Collect a Triad BURST over serial and write to a raw CSV file.

Expected Arduino protocol (see arduino/moonmapper_triad_logger/):
- Send:  BURST <sample_id>
- Receive:
    1 header line starting with: sample_id,burst_index,timestamp_ms,S0_410,...
    20 data rows (CSV) for burst_index 0..19

Output:
- ml/datasets/raw/<sample_id>_triad_raw.csv (default output-dir: ml/datasets/raw)

This script does not install dependencies automatically.
Requires: pyserial
  pip install pyserial

Optional metadata logging:
- Append one row to a metadata CSV (default: ml/datasets/metadata.csv) *after* successful burst.
- The metadata schema is the minimal triad-only schema: sample_id,label_object,label_material,triad_file, ... (no images).
"""

from __future__ import annotations

import argparse
import csv
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple


def _require_pyserial():
    try:
        import serial  # type: ignore

        return serial
    except Exception as exc:  # noqa: BLE001
        raise RuntimeError(
            "pyserial is not installed. Install in your ML venv with: pip install pyserial"
        ) from exc


def _readline_with_timeout(ser, *, timeout_s: float) -> Optional[str]:
    """
    Read one line, waiting up to timeout_s for a newline.
    """
    deadline = time.time() + timeout_s
    buf = bytearray()
    while time.time() < deadline:
        chunk = ser.read(1)
        if chunk:
            if chunk == b"\n":
                try:
                    return buf.decode("utf-8", errors="replace").strip()
                finally:
                    buf.clear()
            if chunk != b"\r":
                buf.extend(chunk)
        else:
            time.sleep(0.01)
    return None


def _send_command_and_expect_ok(ser, *, command: str, timeout_s: float) -> None:
    ser.write((command.strip() + "\n").encode("utf-8"))
    ser.flush()
    line = _readline_with_timeout(ser, timeout_s=timeout_s)
    if line is None:
        raise RuntimeError(f"Timeout waiting for response to command: {command!r}")
    if line.startswith("ERROR"):
        raise RuntimeError(f"Arduino error: {line}")
    if not line.startswith("OK"):
        raise RuntimeError(f"Unexpected Arduino response to {command!r}: {line}")


def collect_burst(
    *,
    port: str,
    baud: int,
    sample_id: str,
    sample_background: bool = False,
    wait_after_sb_s: float = 0.0,
    interactive_after_background: bool = False,
    debug: bool = False,
    timeout_s: float = 5.0,
) -> Tuple[str, List[str]]:
    serial = _require_pyserial()

    try:
        ser = serial.Serial(port=port, baudrate=baud, timeout=0)  # non-blocking reads
    except Exception as exc:  # noqa: BLE001
        raise RuntimeError(f"Could not open serial port {port!r} at {baud}: {exc}") from exc

    with ser:
        # Give Arduino time to reset on serial open (common)
        time.sleep(2.0)
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        if sample_background:
            _send_command_and_expect_ok(ser, command="SB", timeout_s=timeout_s)
            # Small settle time after background sampling
            time.sleep(0.2)
            if wait_after_sb_s > 0:
                print(
                    f"[collect_triad_burst] Vent {wait_after_sb_s:g}s etter SB "
                    f"(legg objekt i sanden før skann)...",
                    file=sys.stderr,
                )
                time.sleep(wait_after_sb_s)
            if interactive_after_background:
                print(
                    "[collect_triad_burst] Bakgrunn tatt (SB). "
                    "Legg objekt ferdig, trykk Enter for å starte BURST...",
                    file=sys.stderr,
                )
                try:
                    input()
                except EOFError:
                    # Non-interactive environment; continue
                    pass

        cmd = f"BURST {sample_id}\n".encode("utf-8")
        ser.write(cmd)
        ser.flush()

        # Some sketches print informational/echo lines before the header.
        # Keep reading until we see the expected header or an ERROR.
        deadline = time.time() + timeout_s
        header: Optional[str] = None
        while time.time() < deadline:
            line = _readline_with_timeout(ser, timeout_s=max(0.1, deadline - time.time()))
            if line is None:
                break
            if debug:
                print(f"[collect_triad_burst][debug] RX: {line}", file=sys.stderr)
            if line.startswith("ERROR"):
                raise RuntimeError(f"Arduino error: {line}")
            if line.startswith("sample_id,burst_index,timestamp_ms,"):
                header = line
                break
            # ignore other lines (e.g. "Mottok kommando: ...")

        if header is None:
            raise RuntimeError(
                "Arduino did not respond with a CSV header (timeout). "
                "Check that the correct sketch is flashed and that BURST prints the CSV header."
            )

        rows: List[str] = []
        for i in range(20):
            # Allow occasional empty/info lines between rows; keep reading until we get a CSV row.
            row_deadline = time.time() + timeout_s
            line: Optional[str] = None
            while time.time() < row_deadline:
                candidate = _readline_with_timeout(ser, timeout_s=max(0.1, row_deadline - time.time()))
                if candidate is None:
                    break
                if not candidate:
                    continue
                if debug:
                    print(f"[collect_triad_burst][debug] RX: {candidate}", file=sys.stderr)
                if candidate.startswith("ERROR"):
                    raise RuntimeError(f"Arduino error during burst: {candidate}")
                # We expect a CSV row starting with sample_id,...
                if "," in candidate:
                    line = candidate
                    break
            if line is None:
                raise RuntimeError(f"Timeout waiting for burst row {i} (expected 20 rows).")
            rows.append(line)

        return header, rows


def write_raw_csv(*, output_path: Path, header_line: str, rows: List[str]) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)

    header = next(csv.reader([header_line]))
    parsed_rows = [next(csv.reader([r])) for r in rows]

    # Basic sanity checks: expected at least 3 + 36 columns
    expected_min_cols = 3 + 36
    if len(header) < expected_min_cols:
        raise RuntimeError(f"Header has too few columns: {len(header)} < {expected_min_cols}")
    for idx, r in enumerate(parsed_rows):
        if len(r) != len(header):
            raise RuntimeError(f"Row {idx} has {len(r)} columns, expected {len(header)}")

    with output_path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        writer.writerows(parsed_rows)


METADATA_HEADER: List[str] = [
    "sample_id",
    "label_object",
    "label_material",
    "triad_file",
    "sand_type",
    "lysforhold",
    "avstand_cm",
    "position_id",
    "angle_id",
    "run_id",
    "diameter_mm",
    "size_group",
    "surface_condition",
    "buried_level",
]


def ensure_metadata_file(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists():
        return
    with path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(METADATA_HEADER)


def append_metadata_row(path: Path, row: Dict[str, str]) -> None:
    ensure_metadata_file(path)
    with path.open("a", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=METADATA_HEADER)
        writer.writerow({k: row.get(k, "") for k in METADATA_HEADER})


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Collect Triad burst over serial and save raw CSV.")
    parser.add_argument("--port", required=True, help="Serial port (e.g. /dev/ttyACM0).")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate (default 115200).")
    parser.add_argument("--sample-id", required=True, help="Sample ID (e.g. S0001).")
    parser.add_argument(
        "--sample-background",
        action="store_true",
        help="Send SB (sample sand background) before BURST.",
    )
    parser.add_argument(
        "--wait-after-sb",
        "--vent-etter-sb",
        dest="wait_after_sb",
        type=float,
        default=0.0,
        metavar="SEC",
        help=(
            "Etter SB: automatisk pause i SEC sekunder før BURST (tidsstyrt; ingen tastetrykk). "
            "Er ikke det samme som --interactive-after-background."
        ),
    )
    parser.add_argument(
        "--interactive-after-background",
        action="store_true",
        help=(
            "Etter SB: stopp og vent på Enter før BURST (manuell start; ingen timer). "
            "Kan kombineres med --wait-after-sb (først timer, så Enter)."
        ),
    )
    parser.add_argument(
        "--output-dir",
        default="ml/datasets/raw",
        help="Output directory (default: ml/datasets/raw).",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Overwrite existing raw file if it already exists.",
    )
    parser.add_argument("--timeout", type=float, default=5.0, help="Line timeout seconds.")
    parser.add_argument(
        "--debug",
        action="store_true",
        help="Print received serial lines to stderr for debugging.",
    )
    parser.add_argument(
        "--metadata-path",
        default="ml/datasets/metadata.csv",
        help="Path to metadata CSV (default: ml/datasets/metadata.csv).",
    )
    parser.add_argument(
        "--append-metadata",
        action="store_true",
        help="Append a metadata row after successful burst collection.",
    )
    parser.add_argument("--label-object", default="", help="Metadata: label_object")
    parser.add_argument("--label-material", default="", help="Metadata: label_material")
    parser.add_argument(
        "--sand-type",
        default="",
        help="Metadata: sand_type (e.g. torr_sand, fuktig_sand).",
    )
    parser.add_argument(
        "--lysforhold",
        default="",
        help="Metadata: lysforhold (f.eks. lampelys, mørk).",
    )
    parser.add_argument(
        "--avstand-cm",
        default="",
        help="Metadata: avstand_cm sensor–objekt i cm (f.eks. 4.0, 4.8).",
    )
    parser.add_argument(
        "--position-id",
        default="P1",
        help="Metadata: position_id (grid/felt, default P1).",
    )
    parser.add_argument(
        "--angle-id",
        default="A1",
        help="Metadata: angle_id / vinkelkode (f.eks. A30, A90 for grader).",
    )
    parser.add_argument("--run-id", default="", help="Metadata: run_id")
    parser.add_argument("--diameter-mm", default="", help="Metadata: diameter_mm")
    parser.add_argument("--size-group", default="", help="Metadata: size_group")
    parser.add_argument(
        "--surface-condition",
        "--surface_condition",
        default="",
        help="Metadata: surface_condition (f.eks. grov, glatt, sand_dekke).",
    )
    parser.add_argument(
        "--buried-level",
        "--buried_level",
        default="",
        help="Metadata: buried_level (f.eks. 0, delvis, full).",
    )
    args = parser.parse_args(argv)

    out_dir = Path(args.output_dir)
    out_path = out_dir / f"{args.sample_id}_triad_raw.csv"

    if out_path.exists() and not args.overwrite:
        print(
            f"[collect_triad_burst] ERROR: raw file already exists: {out_path}. Use --overwrite to replace it.",
            file=sys.stderr,
        )
        return 2

    try:
        header, rows = collect_burst(
            port=args.port,
            baud=int(args.baud),
            sample_id=str(args.sample_id),
            sample_background=bool(args.sample_background),
            wait_after_sb_s=float(args.wait_after_sb),
            interactive_after_background=bool(args.interactive_after_background),
            debug=bool(args.debug),
            timeout_s=float(args.timeout),
        )
        write_raw_csv(output_path=out_path, header_line=header, rows=rows)
    except Exception as exc:  # noqa: BLE001
        print(f"[collect_triad_burst] ERROR: {exc}", file=sys.stderr)
        return 2

    # Only append metadata if burst succeeded and file was written
    if args.append_metadata:
        meta_path = Path(args.metadata_path)
        # Default: triad_file relative to ml/datasets/ when possible
        try:
            triad_rel = out_path.relative_to(Path("ml/datasets"))
            triad_file = str(triad_rel.as_posix())
        except Exception:
            triad_file = str(out_path.as_posix())

        row: Dict[str, str] = {
            "sample_id": str(args.sample_id),
            "label_object": str(args.label_object),
            "label_material": str(args.label_material),
            "triad_file": triad_file,
            "sand_type": str(args.sand_type),
            "lysforhold": str(args.lysforhold),
            "avstand_cm": str(args.avstand_cm),
            "position_id": str(args.position_id),
            "angle_id": str(args.angle_id),
            "run_id": str(args.run_id),
            "diameter_mm": str(args.diameter_mm),
            "size_group": str(args.size_group),
            "surface_condition": str(args.surface_condition),
            "buried_level": str(args.buried_level),
        }

        try:
            append_metadata_row(meta_path, row)
            print(f"[collect_triad_burst] Appended metadata row to: {meta_path}")
        except Exception as exc:  # noqa: BLE001
            print(f"[collect_triad_burst] ERROR: failed to append metadata: {exc}", file=sys.stderr)
            return 2

    print(f"[collect_triad_burst] Saved: {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

