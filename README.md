# Sensor-Data-Fusion – Kalman-Filter Fusion of 2D and 3D Camera Measurements (Master Thesis)

MATLAB simulation of a Kalman filter that estimates the planar pose
(**x, y, θ**) of a moving object/camera by fusing a **2D** and a **3D** measurement
source, plus a velocity (Δx, Δy, Δθ) measurement.

The filter runs in three phases over a 40 s trajectory (Ts = 0.1 s, 401 samples):

| Samples | Phase | Measurements used |
|---|---|---|
| 1–7 | Initialisation | 2D position only (x, y, θ) |
| 8–300 | Fusion | Weighted combination of 2D + 3D position and 3D velocity |
| 301–400 | Dead-reckoning | Velocity only (Δx, Δy, Δθ) |

State model: 9-state constant-acceleration model
`[x y θ vx vy ω ax ay α]`. Ground truth is generated with `lsim` from a
6-state constant-velocity system driven by noisy accelerations.
The script prints the RMS error in x, y and θ and plots estimates,
measurements, error variance (P) and Kalman gain.

## Quick start

1. Open MATLAB (R2016b or newer recommended).
2. `cd` into this repository folder.
3. Run:
   ```matlab
   run_main
   ```
4. Figures are saved to `results/main_fusion/`.

To run a parameter study, open `run_main.m` and set `EXPERIMENT` to one of
`'position_noise'`, `'velocity_noise'`, `'RQ_tuning'` or `'initial_P'`.

**Required toolboxes:** Control System Toolbox (`ss`, `lsim`),
Statistics and Machine Learning Toolbox (`normrnd`).
The camera prototypes in `archive/prototypes/` also need the Computer Vision Toolbox.

**No MATLAB licence?** The code also runs in free [GNU Octave](https://octave.org):
```
pkg install -forge control statistics   % once
pkg load control statistics
run_main
```
(Remove the `pause(0.001)` in the animation loop of `New_filter.m` if it is slow.)

> Results are random (no fixed seed), so RMS values change slightly per run.
> Add `rng(0);` at the top of a script for reproducible numbers.

## Repository structure

```
run_main.m                    Entry point – start here
src/
  main/New_filter.m           FINAL fusion simulator (9-state Kalman filter)
  experiments/
    F_error.m                 Sweep of position-sensor noise (σ = 1…21) → RMS + error plots
    F_errorV.m                Sweep of velocity-sensor noise
    F_RQ.m                    Effect of R / Q on error variance & Kalman gain
    F_iniP.m                  Effect of initial covariance P0
  utils/
    results_dir.m             Resolves/creates results/ sub-folders
    readNPY.m, readNPYheader.m  Read NumPy .npy files (from kwikteam/npy-matlab)
    dataset_dist_analysis.m   Analysis of .npy datasets (paths must be edited)
archive/
  simulator_iterations/       Earlier development versions, chronological:
                              Sim_1 → sim_2 → Sim2 → sim3 → sim4 → sim6–9 → Kalman_2d_example
                              plus first Kalman/fusion experiments (Kalman_Filter, Fusion_Algo, Grid_Search)
  prototypes/                 Camera calibration & camera-motion experiments, "ninja" KF tutorial
third_party/                  External code used as reference (each folder has its own license)
  cams_simulator/             Camera simulator – © 2009 Husam Aldahiyat (MATLAB File Exchange)
  kalman_2d_ball_tracker/     2D ball tracking with Kalman filter – © 2016 Rostam FarrokhNejad
  extended_kalman_filter_3d/  Extended Kalman Filter – © 2012 Alex
results/
  main_fusion/                Output of New_filter.m
  Xerror, Yerror, Therror     Position-noise sweep (F_error.m)
  Xerror_v, Yerror_v, Therror_v  Velocity-noise sweep (F_errorV.m)
  Ini_P, R*,Q*, Varying*      R/Q and P0 studies
  presentation/, new_sim/     Figures used in the thesis presentation
  rms/                        RMS error tables (SD, r_x1, r_y1, r_th1 = phase 2; r_x2… = phase 3)
docs/
  simulator_notes.docx        Notes with selected RMS values
  published_html/             MATLAB "publish" output of early scripts
```

## Changes made when preparing this repository

Only the scripts in `src/` were touched, and only to make them run from any folder:
- `saveas(..., [pwd '/Results/...'])` → `results_dir('<subfolder>')`
- `F_error.m` / `F_errorV.m` now write to separate RMS files instead of both overwriting `RMS_V6.txt`
- `lsim(sys,u,t,x0)` → `lsim(sys,u',t,x0)` (documented input orientation; required by Octave)
- `cov_x(:) = Cov(1,1,:)` → `cov_x = squeeze(Cov(1,1,:))'` (same result, Octave-compatible)

## Notes on the archive

Scripts in `archive/` are kept unchanged for traceability. They still use the
original `[pwd '/Results/...']` save paths, so run them from a folder that
contains a `Results/` directory (or update the `saveas` lines).
