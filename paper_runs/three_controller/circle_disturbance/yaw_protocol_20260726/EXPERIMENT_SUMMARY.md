# Circle disturbance pilot summary — 2026-07-26

This bundle compares Backstepping, BSMC, and pure SMC for paused physical
displacement tests on the Circle trajectory. Errors are camera-derived. The
controller reference clock is frozen while the robot is paused and the
disturbance event is placed at release/resume.

## Selected runs

| Condition | Controller | Selected run | Status |
|---|---|---|---|
| yaw0 | Backstepping | `circle_bs_yaw0_p01_full_lap_manual` | Full run; recovered within 10 s |
| yaw0 | BSMC | `circle_bsmc_yaw0_p03_full_lap_manual` | Full run; recovered within 10 s |
| yaw0 | SMC | `circle_smc_yaw0_p01_full_lap_manual` | Full run; no recovery within 10 s |
| yaw90 | Backstepping | `circle_bs_yaw90_p01_full_lap_manual` | Full run; no recovery within 10 s |
| yaw90 | BSMC | `circle_bsmc_yaw90_p01_full_lap_manual` | Full run; no recovery within 10 s |
| yaw90 | SMC | `circle_smc_yaw90_p06_safety_censored` | Safety-censored failure-to-recover |

## Disturbance metrics

| Condition | Controller | Pre-event error (m) | Peak error (m) | Incremental peak (m) | Peak heading (deg) | IAE10 (m s) | Recovery (s) |
|---|---:|---:|---:|---:|---:|---:|---:|
| yaw0 | Backstepping | 0.0124 | 0.3141 | 0.3017 | 30.81 | 1.3390 | 6.88 |
| yaw0 | BSMC | 0.0121 | 0.2958 | 0.2837 | 28.49 | 1.2789 | 7.24 |
| yaw0 | SMC | 0.0948 | 0.2872 | 0.1924 | 10.89 | 1.3659 | >10 |
| yaw90 | Backstepping | 0.0128 | 0.5394 | 0.5266 | 93.41 | 3.4508 | >10 |
| yaw90 | BSMC | 0.0112 | 0.5531 | 0.5419 | 95.47 | 3.7782 | >10 |
| yaw90 | SMC | 0.0524 | 0.7964 | 0.7440 | 96.66 | 3.5162 | >10, censored |

## Interpretation constraints

- The yaw0 pushes were manual and not equal in magnitude. In particular, the
  SMC incremental displacement was smaller, so its lower peak must not be read
  as better rejection. It still failed the 10 s recovery criterion and had the
  largest full-run position RMSE.
- The yaw90 SMC run was stopped after crossing the predefined safety bound. Its
  full-run RMSE and trajectory length are not directly comparable with the two
  completed runs. Treat it as a right-censored failure-to-recover observation.
- The automatic `valid: true` flag checks technical log validity; it does not
  certify protocol validity. Earlier SMC yaw90 pilots with incorrect yaw,
  missing disturbance events, or aborted placement are excluded.
- These are pilot observations, not independent repeats for inferential
  statistics. Use at least 3–5 independent, controlled repeats per condition
  for paper-level claims.

## Generated assets

Each `yaw0/publication_three` and `yaw90/publication_three` directory contains:

- three-controller trajectory, error, and command figures in PNG and PDF;
- separate `ex`, `ey`, heading, linear-command, and angular-command figures;
- disturbance-aligned recovery and trajectory figures;
- `three_controller_metrics.csv`;
- `three_controller_provenance.json`;
- `circle_three_controller_disturbance_alignment.json`.
