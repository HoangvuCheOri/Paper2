# Best pilot logs selected on 2026-07-25

These files are selected from tuning/pilot runs for inspection and plotting.
They are not the independent validation dataset for the paper.

| Controller | Selected run | Position RMSE | Heading RMSE | Crossing L->R | Crossing R->L |
|---|---|---:|---:|---:|---:|
| BSMC | `eight_bsmc_ff1_center_s2_k2r080_r07` | 2.681 cm | 3.331 deg | 0.294 cm | 1.216 cm |
| Backstepping | `eight_bs_tune_p01` | 3.161 cm | 4.694 deg | 0.923 cm | 1.746 cm |
| SMC | `eight_smc_tune_p03_repeat` | 2.655 cm | 5.320 deg | 0.365 cm | 2.226 cm |

Selection rationale:

- BSMC r07 is the previously accepted balanced figure-eight run, especially at
  the two center-crossing directions. Its raw CSV is the local `190054` source;
  the embedded `/home/thang/...` path in the old summary is stale.
- Backstepping p01 has the lowest position and heading RMSE among the completed
  Backstepping pilot runs.
- SMC p03 repeat has the lowest position and heading RMSE among the completed
  SMC pilot runs and confirms p03 repeatability.
- The interrupted BSMC r02 run from 2026-07-25 is excluded.

