# Test results

Thresholds (config/tests.yaml): VIO ATE ≤ 1.0 %, drift ≤ 1.0 %, rot ≤ 2.0°, init ≤ 6.0 s, seed spread ≤ 0.25; VO ATE ≤ 2.0 %, drift ≤ 3.0 %, rot ≤ 3.0°, |scale−1| ≤ 0.05. Graded profiles: P2, P3; others report only.

## Gates

| Case | G3 flight (track RMS / IMU drift / clearance) | G1 fisheye px | G1 stereo px | G4 disparity px |
|---|---|---|---|---|
| TC-E1-P1 | 0.02 m / 0.04 m / 18.1 m ✅ | 0.078 ✅ | 0.088 ✅ | 1.29 ❌ |
| TC-E1-P2 | 0.04 m / 0.25 m / 9.8 m ✅ | 0.057 ✅ | 0.081 ✅ | 1.35 ❌ |
| TC-E1-P3 | 0.12 m / 0.23 m / 9.1 m ✅ | 0.065 ✅ | 0.076 ✅ | 1.32 ❌ |
| TC-E1-P4 | 0.34 m / 0.23 m / 17.4 m ✅ | 0.074 ✅ | 0.080 ✅ | 1.58 ❌ |
| TC-E1-P5 | 0.11 m / 0.22 m / 53.1 m ✅ | 0.049 ✅ | 0.064 ✅ | 4.13 ✅ |
| TC-E1-P6 | 0.04 m / 0.08 m / 5.1 m ✅ | 0.065 ✅ | – | – |
| TC-E2-P1 | 0.02 m / 0.04 m / 6.0 m ✅ | – | – | – |
| TC-E1-P3-px4 | 0.44 m / 0.19 m / 9.1 m ✅ | 0.157 ❌ | 0.151 ❌ | 1.29 ❌ |

## Estimator runs

| Case | Profile | SUT | Run | Path m | ATE m | ATE % | Final % | Rot ° | Scale | RPE10 % | First pose s | Verdict | Notes |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| TC-E1-P1 | P1 | macvo | fast | 0 | 0.38 | – | – | 0.69 | 0.029 | – | 0.0 | report | no path length (hover),G4 (rescored: hover alignment fix) |
| TC-E1-P1 | P1 | openvins | s101_replay | 0 | 0.26 | – | – | 1.63 | 0.027 | – | 3.6 | report | no path length (hover) |
| TC-E1-P1 | P1 | openvins | s102_replay | 0 | 3.35 | – | – | 0.69 | 0.003 | – | 3.7 | report | no path length (hover) |
| TC-E1-P1 | P1 | openvins | s7_live | 0 | 4.58 | – | – | 0.96 | 0.001 | – | 6.0 | report | no path length (hover) |
| TC-E1-P2 | P2 | macvo | fast | 159 | 1.58 | 0.99 | 2.09 | 4.35 | 1.033 | 5.3 | 0.0 | FAIL | rot,G4 |
| TC-E1-P2 | P2 | openvins | s101_replay | 159 | 53.21 | 33.41 | 66.37 | 10.91 | 0.311 | 183.5 | 2.2 | FAIL | diverged |
| TC-E1-P2 | P2 | openvins | s102_replay | 133 | 28.26 | 21.26 | 70.44 | 39.57 | 0.446 | 109.4 | 19.2 | FAIL | diverged |
| TC-E1-P2 | P2 | openvins | s7_live | 133 | 16.99 | 12.81 | 34.71 | 25.98 | 0.699 | 63.7 | 19.4 | FAIL | diverged |
| TC-E1-P3 | P3 | macvo | fast | 500 | 3.73 | 0.75 | 1.53 | 3.26 | 1.019 | 3.5 | 0.0 | FAIL | rot,G4 |
| TC-E1-P3 | P3 | openvins | s101_replay | 500 | 108.27 | 21.65 | 39.91 | 9.89 | 0.413 | 121.5 | 4.1 | FAIL | diverged |
| TC-E1-P3 | P3 | openvins | s102_replay | 500 | 19.25 | 3.85 | 9.23 | 3.86 | 0.828 | 22.9 | 4.1 | FAIL | ate,drift,rot |
| TC-E1-P3 | P3 | openvins | s7_live | 500 | 63.75 | 12.75 | 23.82 | 7.78 | 0.555 | 73.0 | 4.1 | FAIL | diverged |
| TC-E1-P4 | P4 | macvo | fast | 607 | 1.12 | 0.18 | 0.11 | 1.38 | 0.997 | 2.6 | 0.0 | report | G4 |
| TC-E1-P4 | P4 | openvins | s101_replay | 606 | 1.23 | 0.20 | 0.24 | 0.81 | 0.992 | 6.1 | 4.1 | report |  |
| TC-E1-P4 | P4 | openvins | s102_replay | 606 | 1.17 | 0.19 | 0.33 | 0.82 | 0.995 | 6.0 | 4.1 | report |  |
| TC-E1-P4 | P4 | openvins | s7_live | 606 | 1.07 | 0.18 | 0.26 | 0.80 | 0.990 | 4.3 | 4.1 | report |  |
| TC-E1-P5 | P5 | macvo | fast | 477 | 8.43 | 1.77 | 3.62 | 5.05 | 1.037 | 2.9 | 0.0 | report | drift,rot |
| TC-E1-P5 | P5 | openvins | s101_replay | 456 | 160.69 | 35.23 | 104.87 | 25.45 | 0.280 | 171.8 | 14.4 | FAIL | diverged |
| TC-E1-P5 | P5 | openvins | s102_replay | 456 | 126.15 | 27.66 | 80.73 | 24.95 | 0.341 | 135.7 | 14.4 | FAIL | diverged |
| TC-E1-P5 | P5 | openvins | s7_live | 456 | 3195.89 | 700.70 | 1507.32 | 82.91 | 0.015 | 2765.4 | 14.4 | FAIL | diverged |
| TC-E1-P6 | P6 | openvins | s101_replay | 78 | 26.58 | 34.02 | 36.39 | 145.92 | 1244.613 | 100.5 | 16.2 | FAIL | diverged |
| TC-E1-P6 | P6 | openvins | s102_replay | 78 | 26.58 | 34.02 | 36.40 | 140.19 | 1359.136 | 100.5 | 16.2 | FAIL | diverged |
| TC-E1-P6 | P6 | openvins | s7_live | 78 | 26.59 | 34.03 | 36.41 | 154.66 | 69.957 | 100.5 | 16.2 | FAIL | diverged |
| TC-E1-P3-px4 | P3 | macvo | fast | 501 | 2.33 | 0.46 | 1.06 | 2.47 | 1.000 | 3.0 | 0.0 | PASS | G4 |
| TC-E1-P3-px4 | P3 | openvins | s101_replay | – | – | – | – | – | – | – | – | FAIL | no estimate |
| TC-E1-P3-px4 | P3 | openvins | s102_replay | 472 | 637.13 | 134.85 | 290.95 | 20.62 | 0.094 | 713.4 | 10.4 | FAIL | diverged |
| TC-E1-P3-px4 | P3 | openvins | s7_live | – | – | – | – | – | – | – | – | FAIL | no estimate |

## Run-to-run spread (OpenVINS seeds)

| Case | ATEs m | std/mean | Pass |
|---|---|---|---|
| TC-E1-P1 | 4.58, 0.26, 3.35 | 0.665 | ❌ |
| TC-E1-P2 | 16.99, 53.21, 28.26 | 0.461 | ❌ |
| TC-E1-P3 | 63.75, 108.27, 19.25 | 0.570 | ❌ |
| TC-E1-P4 | 1.07, 1.23, 1.17 | 0.056 | ✅ |
| TC-E1-P5 | 3195.89, 160.69, 126.15 | 1.240 | ❌ |
| TC-E1-P6 | 26.59, 26.58, 26.58 | 0.000 | ✅ |
