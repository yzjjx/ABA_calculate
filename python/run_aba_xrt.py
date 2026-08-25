#!/usr/bin/env python3
"""Run the Vivado XDMA + BRAM ABA design.

The filename is kept for compatibility, but this design is not an XRT kernel:
the FPGA is programmed with a raw .bit file and the host accesses the shared
BRAM through XDMA memory-mapped H2C/C2H channels.

Input text format (one state per row):
    q[0:6], dq[0:6], tau[0:6] -- 18 whitespace-separated float values

Examples
--------
Linux, using the XDMA character devices directly:
    sudo python3 run_aba_xrt.py

Windows, using the xdma_rw.exe supplied with the XDMA reference driver:
    python run_aba_xrt.py --xdma-rw C:\\path\\to\\xdma_rw.exe

Run only the file-format and memory-layout checks, without touching hardware:
    python run_aba_xrt.py --dry-run
"""

from __future__ import annotations

import argparse
import os
import shutil
import struct
import subprocess
import sys
import tempfile
import time
from pathlib import Path
from typing import Dict, Optional, Protocol, Tuple, Union

import numpy as np


# ---------------------------------------------------------------------------
# Hardware memory map. These are AXI byte addresses, not host virtual addresses.
# ---------------------------------------------------------------------------

BRAM_AXI_BASE = 0xC000_0000

COMMAND_OFFSET = 0x0000_0080
STATUS_OFFSET = 0x0000_0084
COUNT_OFFSET = 0x0000_0088
INPUT_OFFSET = 0x0000_0100
OUTPUT_OFFSET = 0x0001_2000

MAX_SAMPLE_COUNT = 1000
DOF = 6
INPUT_VALUES_PER_SAMPLE = 18
OUTPUT_VALUES_PER_SAMPLE = 6
INPUT_BYTES_PER_SAMPLE = INPUT_VALUES_PER_SAMPLE * 4
OUTPUT_BYTES_PER_SAMPLE = OUTPUT_VALUES_PER_SAMPLE * 4

COMMAND_IDLE = 0
COMMAND_START = 1

STATUS_IDLE = 0
STATUS_LOADING = 1
STATUS_RUNNING = 2
STATUS_DONE = 3
STATUS_ERROR = 4

STATUS_NAMES = {
    STATUS_IDLE: "idle",
    STATUS_LOADING: "loading",
    STATUS_RUNNING: "running",
    STATUS_DONE: "done",
    STATUS_ERROR: "error",
}

SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_DATA_DIR = SCRIPT_DIR.parent


class DmaAccess(Protocol):
    def write(self, address: int, data: bytes) -> None:
        ...

    def read(self, address: int, size: int) -> bytes:
        ...

    def close(self) -> None:
        ...


class DirectXdmaAccess:
    """Direct Linux access to the XDMA SGDMA character devices."""

    def __init__(self, h2c_device: str, c2h_device: str) -> None:
        try:
            self._h2c_fd = os.open(h2c_device, os.O_WRONLY)
        except OSError as error:
            raise RuntimeError(f"cannot open H2C device {h2c_device}: {error}") from error

        try:
            self._c2h_fd = os.open(c2h_device, os.O_RDONLY)
        except OSError as error:
            os.close(self._h2c_fd)
            raise RuntimeError(f"cannot open C2H device {c2h_device}: {error}") from error

    @staticmethod
    def _pwrite_all(fd: int, address: int, data: bytes) -> None:
        view = memoryview(data)
        transferred = 0
        while transferred < len(view):
            try:
                if hasattr(os, "pwrite"):
                    count = os.pwrite(fd, view[transferred:], address + transferred)
                else:
                    os.lseek(fd, address + transferred, os.SEEK_SET)
                    count = os.write(fd, view[transferred:])
            except OSError as error:
                raise RuntimeError(
                    f"XDMA H2C write failed at AXI address "
                    f"0x{address + transferred:08X}: {error}"
                ) from error

            if count <= 0:
                raise RuntimeError("XDMA H2C write returned zero bytes")
            transferred += count

    @staticmethod
    def _pread_all(fd: int, address: int, size: int) -> bytes:
        chunks = []
        transferred = 0
        while transferred < size:
            try:
                if hasattr(os, "pread"):
                    chunk = os.pread(fd, size - transferred, address + transferred)
                else:
                    os.lseek(fd, address + transferred, os.SEEK_SET)
                    chunk = os.read(fd, size - transferred)
            except OSError as error:
                raise RuntimeError(
                    f"XDMA C2H read failed at AXI address "
                    f"0x{address + transferred:08X}: {error}"
                ) from error

            if not chunk:
                raise RuntimeError("XDMA C2H read returned zero bytes")
            chunks.append(chunk)
            transferred += len(chunk)

        return b"".join(chunks)

    def write(self, address: int, data: bytes) -> None:
        self._pwrite_all(self._h2c_fd, address, data)

    def read(self, address: int, size: int) -> bytes:
        return self._pread_all(self._c2h_fd, address, size)

    def close(self) -> None:
        if getattr(self, "_h2c_fd", None) is not None:
            os.close(self._h2c_fd)
            self._h2c_fd = None
        if getattr(self, "_c2h_fd", None) is not None:
            os.close(self._c2h_fd)
            self._c2h_fd = None


class XdmaRwAccess:
    """Windows access through the XDMA reference driver's xdma_rw.exe tool."""

    def __init__(self, executable: str, h2c_node: str, c2h_node: str) -> None:
        resolved = shutil.which(executable)
        if resolved is None and Path(executable).is_file():
            resolved = str(Path(executable).resolve())
        if resolved is None:
            raise RuntimeError(
                f"cannot find {executable!r}; pass the full path with --xdma-rw"
            )
        self._executable = resolved
        self._h2c_node = h2c_node
        self._c2h_node = c2h_node

    def _run(self, command: list[str]) -> None:
        completed = subprocess.run(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            check=False,
        )
        if completed.returncode != 0:
            output = completed.stdout.strip()
            raise RuntimeError(
                "xdma_rw failed with exit code "
                f"{completed.returncode}: {' '.join(command)}\n{output}"
            )

    def write(self, address: int, data: bytes) -> None:
        temporary_path: Optional[Path] = None
        try:
            with tempfile.NamedTemporaryFile(suffix=".bin", delete=False) as temporary:
                temporary.write(data)
                temporary_path = Path(temporary.name)

            self._run(
                [
                    self._executable,
                    self._h2c_node,
                    "write",
                    hex(address),
                    "-b",
                    "-f",
                    str(temporary_path),
                ]
            )
        finally:
            if temporary_path is not None:
                temporary_path.unlink(missing_ok=True)

    def read(self, address: int, size: int) -> bytes:
        handle, name = tempfile.mkstemp(suffix=".bin")
        os.close(handle)
        temporary_path = Path(name)
        temporary_path.unlink(missing_ok=True)

        try:
            self._run(
                [
                    self._executable,
                    self._c2h_node,
                    "read",
                    hex(address),
                    "-l",
                    hex(size),
                    "-b",
                    "-f",
                    str(temporary_path),
                ]
            )
            try:
                data = temporary_path.read_bytes()
            except OSError as error:
                raise RuntimeError(f"cannot read xdma_rw output: {error}") from error

            if len(data) != size:
                raise RuntimeError(
                    f"xdma_rw returned {len(data)} bytes, expected {size}"
                )
            return data
        finally:
            temporary_path.unlink(missing_ok=True)

    def close(self) -> None:
        pass


def auto_int(value: str) -> int:
    try:
        return int(value, 0)
    except ValueError as error:
        raise argparse.ArgumentTypeError(f"invalid integer: {value}") from error


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run 1..1000 floating-point ABA states through XDMA and BRAM."
    )
    parser.add_argument(
        "--input",
        type=Path,
        default=PROJECT_DATA_DIR / "in_txt" / "aba_input.txt",
        help="Input text file: each row is q[6], dq[6], tau[6].",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=PROJECT_DATA_DIR / "out_txt" / "aba_output_fpga.txt",
        help="Text file receiving FPGA ddq results.",
    )
    parser.add_argument(
        "--timing-output",
        type=Path,
        default=PROJECT_DATA_DIR / "out_txt" / "aba_timing_fpga.txt",
        help="Text file receiving transfer and execution timing.",
    )
    parser.add_argument(
        "--reference",
        type=Path,
        help="Optional floating-point reference output for error statistics.",
    )
    parser.add_argument(
        "--count",
        type=int,
        default=1000,
        help="Number of input rows to calculate (1..1000).",
    )
    parser.add_argument(
        "--repeat",
        type=int,
        default=1,
        help="Number of complete FPGA calculations to time.",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=2.0,
        help="Timeout in seconds for idle/done status polling.",
    )
    parser.add_argument(
        "--poll-interval",
        type=float,
        default=0.001,
        help="Delay in seconds between status reads.",
    )
    parser.add_argument(
        "--base-address",
        type=auto_int,
        default=BRAM_AXI_BASE,
        help="XDMA AXI BRAM base address (default: 0xC0000000).",
    )
    parser.add_argument(
        "--backend",
        choices=("auto", "direct", "xdma-rw"),
        default="auto",
        help="auto selects xdma-rw on Windows and direct devices elsewhere.",
    )
    parser.add_argument("--h2c", default="/dev/xdma0_h2c_0")
    parser.add_argument("--c2h", default="/dev/xdma0_c2h_0")
    parser.add_argument("--xdma-rw", default="xdma_rw.exe")
    parser.add_argument("--h2c-node", default="h2c_0")
    parser.add_argument("--c2h-node", default="c2h_0")
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Validate and pack input without accessing the FPGA.",
    )
    args = parser.parse_args()

    if not 1 <= args.count <= MAX_SAMPLE_COUNT:
        parser.error(f"--count must be in the range 1..{MAX_SAMPLE_COUNT}")
    if args.repeat <= 0:
        parser.error("--repeat must be positive")
    if args.timeout <= 0:
        parser.error("--timeout must be positive")
    if args.poll_interval < 0:
        parser.error("--poll-interval must be non-negative")
    return args


def load_inputs(path: Path, count: int) -> np.ndarray:
    try:
        values = np.loadtxt(path, dtype=np.float64)
    except (OSError, ValueError) as error:
        raise RuntimeError(f"cannot read input file {path}: {error}") from error

    values = np.atleast_2d(values)
    if values.ndim != 2 or values.shape[1] != INPUT_VALUES_PER_SAMPLE:
        raise RuntimeError(
            f"input must have 18 columns, but its shape is {values.shape}"
        )
    if values.shape[0] < count:
        raise RuntimeError(
            f"input contains {values.shape[0]} rows, but --count is {count}"
        )

    selected = values[:count, :]
    if not np.isfinite(selected).all():
        raise RuntimeError("selected input rows contain NaN or infinity")

    # The HLS data_t is float, so the BRAM contains IEEE-754 binary32 words.
    return np.ascontiguousarray(selected, dtype="<f4")


def open_dma(args: argparse.Namespace) -> DmaAccess:
    backend = args.backend
    if backend == "auto":
        backend = "xdma-rw" if os.name == "nt" else "direct"

    if backend == "direct":
        return DirectXdmaAccess(args.h2c, args.c2h)
    return XdmaRwAccess(args.xdma_rw, args.h2c_node, args.c2h_node)


def address(base: int, offset: int) -> int:
    return base + offset


def write_u32(dma: DmaAccess, axi_address: int, value: int) -> None:
    dma.write(axi_address, struct.pack("<I", value & 0xFFFF_FFFF))


def read_u32(dma: DmaAccess, axi_address: int) -> int:
    return struct.unpack("<I", dma.read(axi_address, 4))[0]


def wait_for_status(
    dma: DmaAccess,
    status_address: int,
    accepted_statuses: Tuple[int, ...],
    timeout: float,
    poll_interval: float,
) -> int:
    deadline = time.monotonic() + timeout
    previous: Optional[int] = None

    while True:
        status = read_u32(dma, status_address)
        if status != previous:
            print(f"FPGA status: {status} ({STATUS_NAMES.get(status, 'unknown')})")
            previous = status

        if status in accepted_statuses:
            return status
        if status == STATUS_ERROR:
            raise RuntimeError("FPGA controller reported STATUS_ERROR (4)")
        if time.monotonic() >= deadline:
            name = STATUS_NAMES.get(status, "unknown")
            raise RuntimeError(
                f"timeout waiting for FPGA status; last status={status} ({name})"
            )
        if poll_interval:
            time.sleep(poll_interval)


def prepare_idle(dma: DmaAccess, args: argparse.Namespace) -> None:
    command_address = address(args.base_address, COMMAND_OFFSET)
    status_address = address(args.base_address, STATUS_OFFSET)

    write_u32(dma, command_address, COMMAND_IDLE)
    wait_for_status(
        dma,
        status_address,
        (STATUS_IDLE,),
        args.timeout,
        args.poll_interval,
    )


def execute_once(dma: DmaAccess, args: argparse.Namespace) -> float:
    command_address = address(args.base_address, COMMAND_OFFSET)
    status_address = address(args.base_address, STATUS_OFFSET)

    prepare_idle(dma, args)

    start_ns = time.perf_counter_ns()
    write_u32(dma, command_address, COMMAND_START)
    wait_for_status(
        dma,
        status_address,
        (STATUS_DONE,),
        args.timeout,
        args.poll_interval,
    )
    end_ns = time.perf_counter_ns()
    return (end_ns - start_ns) / 1_000.0


def write_outputs(path: Path, values: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    np.savetxt(path, values, fmt="%.9g")


def write_timing(path: Path, timing: Dict[str, Union[int, float, str]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="\n") as output:
        for key, value in timing.items():
            if isinstance(value, float):
                output.write(f"{key} {value:.6f}\n")
            else:
                output.write(f"{key} {value}\n")


def compare_reference(path: Path, actual: np.ndarray, count: int) -> Dict[str, float]:
    try:
        reference = np.loadtxt(path, dtype=np.float64)
    except (OSError, ValueError) as error:
        raise RuntimeError(f"cannot read reference file {path}: {error}") from error

    reference = np.atleast_2d(reference)
    if reference.ndim != 2 or reference.shape[1] != DOF:
        raise RuntimeError(
            f"reference must have 6 columns, but its shape is {reference.shape}"
        )
    if reference.shape[0] < count:
        raise RuntimeError(
            f"reference contains {reference.shape[0]} rows, expected at least {count}"
        )

    error = actual.astype(np.float64) - reference[:count, :]
    return {
        "max_abs_error": float(np.max(np.abs(error))),
        "mean_abs_error": float(np.mean(np.abs(error))),
        "rmse": float(np.sqrt(np.mean(error * error))),
    }


def main() -> int:
    args = parse_args()
    input_values = load_inputs(args.input, args.count)
    input_bytes = input_values.tobytes(order="C")

    expected_input_size = args.count * INPUT_BYTES_PER_SAMPLE
    if len(input_bytes) != expected_input_size:
        raise RuntimeError(
            f"internal input size error: {len(input_bytes)} != {expected_input_size}"
        )

    print(f"Input:          {args.input}")
    print(f"Sample count:   {args.count}")
    print(f"Input bytes:    {len(input_bytes)}")
    print(f"Input AXI addr: 0x{address(args.base_address, INPUT_OFFSET):08X}")
    print(f"Output AXI addr:0x{address(args.base_address, OUTPUT_OFFSET):08X}")
    print("Data format:    IEEE-754 float32, little-endian")

    if args.dry_run:
        print("Dry run completed; no XDMA device was opened.")
        return 0

    dma = open_dma(args)
    try:
        prepare_idle(dma, args)

        h2c_start_ns = time.perf_counter_ns()
        dma.write(address(args.base_address, INPUT_OFFSET), input_bytes)
        write_u32(dma, address(args.base_address, COUNT_OFFSET), args.count)
        h2c_end_ns = time.perf_counter_ns()

        execution_times_us = []
        for run_index in range(args.repeat):
            print(f"Starting FPGA run {run_index + 1}/{args.repeat}")
            execution_times_us.append(execute_once(dma, args))

        output_size = args.count * OUTPUT_BYTES_PER_SAMPLE
        c2h_start_ns = time.perf_counter_ns()
        output_bytes = dma.read(
            address(args.base_address, OUTPUT_OFFSET), output_size
        )
        c2h_end_ns = time.perf_counter_ns()

        # Clear the command after preserving the completed output data.
        write_u32(
            dma,
            address(args.base_address, COMMAND_OFFSET),
            COMMAND_IDLE,
        )
    finally:
        dma.close()

    outputs = np.frombuffer(output_bytes, dtype="<f4").copy().reshape(
        args.count, OUTPUT_VALUES_PER_SAMPLE
    )
    if not np.isfinite(outputs).all():
        bad_count = int(np.count_nonzero(~np.isfinite(outputs)))
        raise RuntimeError(f"FPGA output contains {bad_count} NaN/Inf values")

    write_outputs(args.output, outputs)

    execution_array = np.asarray(execution_times_us, dtype=np.float64)
    h2c_us = (h2c_end_ns - h2c_start_ns) / 1_000.0
    c2h_us = (c2h_end_ns - c2h_start_ns) / 1_000.0
    execution_mean_us = float(execution_array.mean())

    timing: Dict[str, Union[int, float, str]] = {
        "sample_count": args.count,
        "repeat": args.repeat,
        "backend": args.backend,
        "bram_axi_base": f"0x{args.base_address:08X}",
        "h2c_us": h2c_us,
        "command_to_done_mean_us": execution_mean_us,
        "command_to_done_min_us": float(execution_array.min()),
        "command_to_done_max_us": float(execution_array.max()),
        "c2h_us": c2h_us,
        "observed_us_per_sample": execution_mean_us / args.count,
        "observed_samples_per_second": args.count * 1_000_000.0 / execution_mean_us,
    }

    if args.reference is not None:
        error_statistics = compare_reference(args.reference, outputs, args.count)
        timing.update(error_statistics)
        print(
            "Reference error: "
            f"max={error_statistics['max_abs_error']:.9g}, "
            f"mean={error_statistics['mean_abs_error']:.9g}, "
            f"RMSE={error_statistics['rmse']:.9g}"
        )

    write_timing(args.timing_output, timing)

    print(f"H2C input transfer:       {h2c_us:.3f} us")
    print(
        "Command-to-done:         "
        f"mean {execution_mean_us:.3f} us, "
        f"min {execution_array.min():.3f} us, "
        f"max {execution_array.max():.3f} us"
    )
    print(f"C2H output transfer:      {c2h_us:.3f} us")
    print(f"Observed time/state:      {execution_mean_us / args.count:.6f} us")
    print(
        "Observed throughput:      "
        f"{args.count * 1_000_000.0 / execution_mean_us:.3f} states/s"
    )
    print(f"Output:                   {args.output}")
    print(f"Timing:                   {args.timing_output}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except RuntimeError as error:
        print(f"ERROR: {error}", file=sys.stderr)
        raise SystemExit(1)
