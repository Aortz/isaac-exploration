# OpenVINS with four mono cameras: repeatability investigation (2026-10-31)

All runs below replay the same recorded ROS 2 bag: the TC-REG Cesium Marina Bay loop with the Pegasus Iris, 846 m, 105 s. They use the same IMU stream (`imu.csv`) and score the same way (4-DoF ATE, % of path). Each row is one OpenVINS run.

| OpenVINS build | Replay speed | ATE % of path (scale) per run |
|---|---|---|
| Upstream (no init patch) | 0.5x | 1.74 (1.02), 1.31 (1.01) |
| Patch A, drop duplicate camera times in the init clone queue | 0.5x | 8.31 (2.44), 1.93 (1.11) |
| Patch B, skip only the clone equal to the state time (**kept**) | 0.5x | 1.52 (0.95), 8.32 (2.44) |
| Patch B | 0.2x | 8.31, 8.32, 8.32 (all 2.44) |
| Patch B, front camera only | 0.5x | 4.18, 4.18, 4.18 (0.77): identical |
| Patch B + "feed a frame only when complete" | 0.5x / 0.2x | 4.34, 99.8, 99.8 / 4.17 |
| Patch B + "merge the 4 cameras into one measurement" (`openvins_sync_cameras_patch.py`) | 0.5x / 0.2x | 12.0, 12.6, no output / no output |
| Live run on 2026-10-30 (rendering at about 0.17x, upstream + IMU QoS fix) | live | 0.89 (1.00) |

## Findings

- **One camera is repeatable; four are not.** Every front-camera run gives the identical trajectory, while four-camera runs vary between runs on the same bag. The variation comes from how OpenVINS' ROS 2 node takes in four separate mono streams.
  - Each image is its own measurement.
  - Images and IMU samples are processed in whatever order the ROS 2 executor delivers them.
  - Images of one frame can be fed in different orders or dropped as "out of order".
- **The solutions are bimodal.** A run lands in a good solution (about 1.3–1.9 %) or a bad one (8.3 %, scale 2.44, reproduced to the millimetre). Slower replay lands in the bad one more often.
- **Patch A was not the cause.** The regression first looked like it came from Patch A, but upstream and Patch B reach the same bad solution.
  - Patch B is kept because it removes the `exit()` at initialisation that killed the hover case. A clone equal to the state time is skipped instead of crashing.
  - Patch A is gone.
- **Two order-fixing patches made things worse.** Requiring complete frames, or merging the cameras into one measurement, hurt the results. OpenVINS' multi-camera core appears to expect the per-camera feed it was written for.

## Consequences for the test plan

- **Reporting:** OpenVINS results with four cameras are reported over three runs per case (seeds), and the run-to-run spread is a reported metric (`vio_seed_spread`). Single numbers should not be quoted.
- **A1 ablation (front camera only):** this is the repeatable configuration, though less accurate on this flight (4.2 %).
- **Proper fix (not done):** synchronised multi-camera input inside OpenVINS' VioManager (one clone per frame, every camera's features attached to it), or a different multi-camera VIO such as VINS-Fusion or Kimera with matching camera synchronisation.
