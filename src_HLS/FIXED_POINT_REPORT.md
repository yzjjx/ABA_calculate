# ABA HLS fixed-point conversion and accuracy report

## Datapath formats

All synthesizable arithmetic is fixed point. Every external batch-buffer
element remains 32 bits wide; only the binary-point position differs by signal.

| Signal/type | Format | Range / purpose |
|---|---:|---|
| `joint_pos_t` | `ap_fixed<32,4>` | `q`, deployment range `[-pi, pi]` |
| `joint_vel_t` | `ap_fixed<32,4>` | `dq`, deployment range `[-2, 2]` |
| `joint_tau_t` | `ap_fixed<32,8>` | `tau`, deployment range `[-10, 10]` |
| `data_t` | `ap_fixed<32,16>` | `ddq`, range `[-32768, 32767.99998]` |
| `transform_t` | `ap_fixed<22,2>` | rotation and compact spatial transform |
| `kinematic_t` | `ap_fixed<28,8>` | velocity and bias acceleration |
| `inertia_t` | `ap_fixed<32,8>` | spatial/articulated inertia; 24 fractional bits |
| `force_t` | `ap_fixed<32,12>` | momentum, bias force and joint force |
| `inverse_t` | `ap_fixed<34,14>` | reciprocal joint-space inertia |

All computation types use convergent rounding and saturation. The trigonometric
path uses fixed-point `hls::sincos` with `angle_t=ap_fixed<24,4>` and
`trig_t=ap_fixed<22,2>`. The old floating-point allocation pragmas were removed.

The input and output AXI words are all still 32 bits, but host-side packing must
use the binary-point positions above. Raw integer scaling factors are:

- `q`, `dq`: `2^28`
- `tau`: `2^24`
- `ddq`: `2^16`

## Software accuracy verification

Command (ordinary native C++ simulation; it does not invoke Vitis HLS):

```powershell
./src_HLS/run_fixed_accuracy.ps1
```

Reference: the untouched single-precision implementation in `src/`, compiled
under renamed host-only symbols by `float_reference.cpp`.

Test data:

- 1000 deterministic random states from `in_txt/aba_input.txt`:
  `q in [-pi,pi]`, `dq in [-2,2]`, `tau in [-10,10]`.
- 65 boundary states: the all-zero state and all 64 sign combinations at
  `|q|=pi`, `|dq|=2`, `|tau|=10`.

Results from 2026-08-25:

| Set | Values | MAE | RMSE | P99 absolute | Maximum absolute | Mean normalized |
|---|---:|---:|---:|---:|---:|---:|
| Random | 6000 | 0.1058489 | 0.2843990 | 1.1122894 | 1.2118683 | 0.005935% |
| Boundary | 390 | 0.2006250 | 0.4830775 | 1.2218323 | 1.2252502 | 0.002217% |

Random-set per-joint maximum absolute error:

| Joint | Maximum absolute error |
|---:|---:|
| 1 | 0.0017166 |
| 2 | 0.0013580 |
| 3 | 0.0030365 |
| 4 | 0.0133667 |
| 5 | 0.0817566 |
| 6 | 1.2118683 |

The random reference maximum magnitude was `22558.4746094`; the boundary-set
maximum was `23394.9628906`. The latter leaves about 28.6% headroom to the
`data_t` positive/negative limits. No output saturation was observed. The
native test quantizes sine/cosine to the exact fixed CORDIC output type;
synthesis uses `hls::sincos` directly.

## Files for HLS

Use `ABA` or `ABA_batch` as the top function and include `src_HLS` plus the
Vitis HLS include directory. Do not add the following host-only verification
files to synthesis:

- `fixed_accuracy_test.cpp`
- `float_reference.cpp`
- `generate_input.cpp`
- `I_space_out.cpp`
- `run_fixed_accuracy.ps1`

`I_space.cpp` is also unnecessary because `I_spa` is a precomputed constant.
