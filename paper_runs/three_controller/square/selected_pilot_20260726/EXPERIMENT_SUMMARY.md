# Square three-controller pilot summary

Generated on 2026-07-26 from selected existing real-robot logs. No disturbance
run is included because disturbance testing is reserved for the Circle
trajectory.

## Selected runs

| Controller | Run | Class | Laps |
|---|---|---|---:|
| BSMC | `final_square_bsmc_3laps` | Final | 3 |
| Backstepping | `final_square_2m_2laps_bs_r01` | Final | 2 |
| SMC | `square_smc_tune_p04_aggressive` | Tuning pilot | 1 |

## Results

| Controller | Position RMSE | Straight lateral RMSE | Straight bias | Straight max | Straight heading RMSE |
|---|---:|---:|---:|---:|---:|
| BSMC | 3.04 cm | 3.26 cm | 3.07 cm | 5.53 cm | 2.24 deg |
| Backstepping | 3.22 cm | 3.44 cm | 3.28 cm | 5.35 cm | 2.11 deg |
| SMC | 8.95 cm | 8.94 cm | 6.79 cm | 19.19 cm | 7.90 deg |

BSMC has the lowest selected position and straight lateral RMSE. Backstepping
is close and has the lowest straight heading RMSE. The selected SMC pilot is
substantially worse, especially immediately after sharp corners.

## Interpretation limits

- BSMC and Backstepping are older final runs; SMC is a 2026-07-26 tuning run.
- The selected runs contain different lap counts.
- Therefore this bundle is a pilot/diagnostic comparison, not a final
  publication validation set.
- Tuning data must not be relabeled as validation data.
- A final statistical comparison requires new locked-parameter repeats under
  the same robot, firmware, floor, camera, EKF, start-pose, speed, and lap
  conditions.

## Outputs

- `publication_three/`: combined trajectory, error, and command figures in PNG
  and PDF.
- `publication_three/three_controller_metrics.csv`: aggregate metrics from the
  standard three-controller renderer.
- `selected_square_metrics.csv`: selected-run metrics plus straight-edge
  diagnostics.
- `square_reports/`: per-controller straight-edge report, JSON summary, and
  edge-level CSV.
