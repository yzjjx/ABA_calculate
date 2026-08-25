#!/usr/bin/env python3
"""Run the 1000-state fixed-point ABA_batch kernel through Linux XRT.

The four accelerator buffers are all 32-bit words, with these binary points:
    q   : ap_fixed<32, 4>  (scale 2**28)
    dq  : ap_fixed<32, 4>  (scale 2**28)
    tau : ap_fixed<32, 8>  (scale 2**24)
    ddq : ap_fixed<32,16>  (scale 2**16)

Example:
    python3 run_aba_xrt.py --xclbin build/aba.xclbin
"""

import argparse
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Tuple, Union

import numpy as np


SAMPLE_COUNT = 1000
DOF = 6
VALUES_PER_ROW = DOF * 3

Q_FRACTION_BITS = 28
DQ_FRACTION_BITS = 28
TAU_FRACTION_BITS = 24
DDQ_FRACTION_BITS = 16

INT32_MIN = -(1 << 31)
INT32_MAX = (1 << 31) - 1


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Run 1000 ABA states on the fixed-point ABA_batch FPGA kernel "
            "and report transfer, kernel, and end-to-end timing."
        )
    )
    parser.add_argument(
        "--xclbin",
        type=Path,
        help="Path to the compiled XRT .xclbin file (required unless --dry-run).",
    )
    parser.add_argument(
        "--input",
        type=Path,
        default=Path("in_txt/aba_input.txt"),
        help="1000-row input file containing q[6], dq[6], tau[6].",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("out_txt/aba_output_fpga.txt"),
        help="Decoded ddq output text file.",
    )
    parser.add_argument(
        "--timing-output",
        type=Path,
        default=Path("out_txt/aba_timing_fpga.txt"),
        help="Timing summary text file.",
    )
    parser.add_argument("--device", type=int, default=0, help="XRT device index.")
    parser.add_argument(
        "--kernel", default="ABA_batch", help="Kernel name stored in the xclbin."
    )
    parser.add_argument(
        "--warmup",
        type=int,
        default=1,
        help="Unmeasured kernel executions before timing.",
    )
    parser.add_argument(
        "--repeat",
        type=int,
        default=1,
        help="Number of measured kernel executions; statistics are reported.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Only read and fixed-point-pack the input; do not access XRT.",
    )
    args = parser.parse_args()

    if not args.dry_run and args.xclbin is None:
        parser.error("--xclbin is required unless --dry-run is used")
    if args.warmup < 0:
        parser.error("--warmup must be non-negative")
    if args.repeat <= 0:
        parser.error("--repeat must be positive")
    return args


def load_inputs(path: Path) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    try:
        values = np.loadtxt(path, dtype=np.float64)
    except (OSError, ValueError) as error:
        raise RuntimeError(f"cannot read input file {path}: {error}") from error

    expected_shape = (SAMPLE_COUNT, VALUES_PER_ROW)
    if values.shape != expected_shape:
        raise RuntimeError(
            f"input must have shape {expected_shape}, but got {values.shape}"
        )
    if not np.isfinite(values).all():
        raise RuntimeError("input contains NaN or infinity")

    q = np.ascontiguousarray(values[:, 0:DOF])
    dq = np.ascontiguousarray(values[:, DOF : 2 * DOF])
    tau = np.ascontiguousarray(values[:, 2 * DOF : 3 * DOF])
    return q, dq, tau


def pack_fixed(values: np.ndarray, fraction_bits: int, name: str) -> np.ndarray:
    """Quantize with convergent rounding and saturation, matching ap_fixed."""
    scale = float(1 << fraction_bits)
    scaled = np.rint(values * scale)  # round-to-nearest, ties-to-even
    clipped = np.clip(scaled, INT32_MIN, INT32_MAX)
    saturation_count = int(np.count_nonzero(scaled != clipped))
    if saturation_count:
        raise RuntimeError(
            f"{name}: {saturation_count} values exceed the 32-bit fixed range"
        )
    return np.ascontiguousarray(clipped, dtype=np.int32).reshape(-1)


def decode_fixed(raw: np.ndarray, fraction_bits: int) -> np.ndarray:
    return raw.astype(np.float64).reshape(SAMPLE_COUNT, DOF) / float(
        1 << fraction_bits
    )


def import_pyxrt() -> Any:
    try:
        import pyxrt  # type: ignore
    except ImportError as error:
        raise RuntimeError(
            "cannot import pyxrt; install/source the Xilinx XRT environment "
            "(commonly: source /opt/xilinx/xrt/setup.sh)"
        ) from error
    return pyxrt


def allocate_bo(pyxrt: Any, device: Any, kernel: Any, arg_index: int, size: int) -> Any:
    return pyxrt.bo(
        device,
        size,
        pyxrt.bo.flags.normal,
        kernel.group_id(arg_index),
    )


def mapped_int32_view(buffer_object: Any, count: int) -> np.ndarray:
    return np.frombuffer(buffer_object.map(), dtype=np.int32, count=count)


def sync_to_device(pyxrt: Any, buffer_object: Any, size: int) -> None:
    buffer_object.sync(
        pyxrt.xclBOSyncDirection.XCL_BO_SYNC_BO_TO_DEVICE, size, 0
    )


def sync_from_device(pyxrt: Any, buffer_object: Any, size: int) -> None:
    buffer_object.sync(
        pyxrt.xclBOSyncDirection.XCL_BO_SYNC_BO_FROM_DEVICE, size, 0
    )


def write_outputs(path: Path, ddq: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    np.savetxt(path, ddq, fmt="%.9f")


def write_timing(
    path: Path, timing: Dict[str, Union[float, int, str]]
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="\n") as output:
        for key, value in timing.items():
            if isinstance(value, float):
                output.write(f"{key} {value:.6f}\n")
            else:
                output.write(f"{key} {value}\n")


def main() -> int:
    args = parse_args()

    q, dq, tau = load_inputs(args.input)
    q_raw = pack_fixed(q, Q_FRACTION_BITS, "q")
    dq_raw = pack_fixed(dq, DQ_FRACTION_BITS, "dq")
    tau_raw = pack_fixed(tau, TAU_FRACTION_BITS, "tau")

    print(f"Loaded {SAMPLE_COUNT} states from {args.input}")
    print(
        "Packed formats: q=Q4.28, dq=Q4.28, tau=Q8.24, ddq=Q16.16 "
        "(all signed 32-bit)"
    )

    if args.dry_run:
        print("Dry run completed; XRT was not accessed.")
        print(
            f"Raw ranges: q=[{q_raw.min()}, {q_raw.max()}], "
            f"dq=[{dq_raw.min()}, {dq_raw.max()}], "
            f"tau=[{tau_raw.min()}, {tau_raw.max()}]"
        )
        return 0

    if not args.xclbin.is_file():
        raise RuntimeError(f"xclbin does not exist: {args.xclbin}")

    pyxrt = import_pyxrt()
    device = pyxrt.device(args.device)
    xclbin = pyxrt.xclbin(str(args.xclbin))
    uuid = device.load_xclbin(xclbin)

    try:
        kernel = pyxrt.kernel(device, uuid, args.kernel)
    except Exception as error:
        available = []
        try:
            available = [item.get_name() for item in xclbin.get_kernels()]
        except Exception:
            pass
        suffix = f"; available kernels: {available}" if available else ""
        raise RuntimeError(f"cannot open kernel {args.kernel!r}{suffix}") from error

    element_count = SAMPLE_COUNT * DOF
    buffer_size = element_count * np.dtype(np.int32).itemsize

    q_bo = allocate_bo(pyxrt, device, kernel, 0, buffer_size)
    dq_bo = allocate_bo(pyxrt, device, kernel, 1, buffer_size)
    tau_bo = allocate_bo(pyxrt, device, kernel, 2, buffer_size)
    ddq_bo = allocate_bo(pyxrt, device, kernel, 3, buffer_size)

    np.copyto(mapped_int32_view(q_bo, element_count), q_raw)
    np.copyto(mapped_int32_view(dq_bo, element_count), dq_raw)
    np.copyto(mapped_int32_view(tau_bo, element_count), tau_raw)

    transfer_start_ns = time.perf_counter_ns()
    sync_to_device(pyxrt, q_bo, buffer_size)
    sync_to_device(pyxrt, dq_bo, buffer_size)
    sync_to_device(pyxrt, tau_bo, buffer_size)
    h2d_end_ns = time.perf_counter_ns()

    for _ in range(args.warmup):
        warmup_run = kernel(q_bo, dq_bo, tau_bo, ddq_bo)
        warmup_run.wait()

    kernel_times_us: List[float] = []
    for _ in range(args.repeat):
        kernel_start_ns = time.perf_counter_ns()
        measured_run = kernel(q_bo, dq_bo, tau_bo, ddq_bo)
        measured_run.wait()
        kernel_end_ns = time.perf_counter_ns()
        kernel_times_us.append((kernel_end_ns - kernel_start_ns) / 1_000.0)

    d2h_start_ns = time.perf_counter_ns()
    sync_from_device(pyxrt, ddq_bo, buffer_size)
    d2h_end_ns = time.perf_counter_ns()

    ddq_raw = mapped_int32_view(ddq_bo, element_count).copy()
    ddq = decode_fixed(ddq_raw, DDQ_FRACTION_BITS)
    write_outputs(args.output, ddq)

    h2d_us = (h2d_end_ns - transfer_start_ns) / 1_000.0
    d2h_us = (d2h_end_ns - d2h_start_ns) / 1_000.0
    kernel_array = np.asarray(kernel_times_us, dtype=np.float64)
    kernel_mean_us = float(kernel_array.mean())
    kernel_min_us = float(kernel_array.min())
    kernel_max_us = float(kernel_array.max())
    end_to_end_us = h2d_us + kernel_mean_us + d2h_us

    timing: Dict[str, Union[float, int, str]] = {
        "sample_count": SAMPLE_COUNT,
        "device_index": args.device,
        "kernel": args.kernel,
        "warmup_runs": args.warmup,
        "measured_runs": args.repeat,
        "h2d_us": h2d_us,
        "kernel_mean_us": kernel_mean_us,
        "kernel_min_us": kernel_min_us,
        "kernel_max_us": kernel_max_us,
        "d2h_us": d2h_us,
        "end_to_end_mean_us": end_to_end_us,
        "kernel_us_per_state": kernel_mean_us / SAMPLE_COUNT,
        "kernel_states_per_second": SAMPLE_COUNT * 1_000_000.0 / kernel_mean_us,
    }
    write_timing(args.timing_output, timing)

    print(f"H2D transfer:          {h2d_us:.3f} us")
    print(
        f"Kernel ({args.repeat} run(s)): mean {kernel_mean_us:.3f} us, "
        f"min {kernel_min_us:.3f} us, max {kernel_max_us:.3f} us"
    )
    print(f"D2H transfer:          {d2h_us:.3f} us")
    print(f"End-to-end mean:       {end_to_end_us:.3f} us")
    print(f"Kernel time/state:     {kernel_mean_us / SAMPLE_COUNT:.6f} us")
    print(
        "Kernel throughput:     "
        f"{SAMPLE_COUNT * 1_000_000.0 / kernel_mean_us:.3f} states/s"
    )
    print(f"Output:                 {args.output}")
    print(f"Timing:                 {args.timing_output}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except RuntimeError as error:
        print(f"ERROR: {error}", file=sys.stderr)
        raise SystemExit(1)
