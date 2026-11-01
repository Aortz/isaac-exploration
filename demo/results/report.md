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
| TC-E1-P6 | 0.04 m / 0.08 m / 5.1 m ✅ | 0.065 ✅ | 0.089 ✅ | 5.38 ✅ |
| TC-E1-P7-low_sun | 0.12 m / 0.23 m / 9.1 m ✅ | 0.083 ✅ | 0.083 ✅ | 1.03 ❌ |
| TC-E1-P7-overcast | 0.12 m / 0.23 m / 9.1 m ✅ | 0.076 ✅ | 0.076 ✅ | 1.03 ❌ |
| TC-E2-P1 | 0.02 m / 0.04 m / 6.0 m ✅ | 0.163 ❌ | 0.442 ❌ | 7.11 ✅ |
| TC-E2-P2 | 0.04 m / 0.03 m / 3.2 m ✅ | 0.151 ❌ | 0.316 ❌ | 6.65 ✅ |
| TC-E2-P3 | 0.08 m / 0.06 m / 1.8 m ✅ | 0.201 ❌ | 0.873 ❌ | 6.68 ✅ |
| TC-E1-P3-px4 | 0.45 m / 0.01 m / 9.1 m ✅ | 0.064 ✅ | 0.079 ✅ | 1.31 ❌ |
| TC-EF-P3 | 0.10 m / 0.03 m / 11.1 m ✅ | 0.101 ❌ | 0.113 ❌ | 1.39 ❌ |
| TC-ED-P3 | 0.09 m / 0.01 m / 21.3 m ✅ | 0.025 ✅ | 0.021 ✅ | 0.00 ❌ |
| TC-EF2-P3 | 0.10 m / 0.03 m / 7.3 m ✅ | – | – | – |
| TC-SG-P3 | 0.07 m / 0.01 m / 7.9 m ✅ | – | – | – |
| TC-REG | 0.24 m / 0.14 m / – m ✅ | 0.037 ✅ | 0.050 ✅ | 0.59 ❌ |

## Estimator runs

| Case | Profile | SUT | Run | Path m | ATE m | ATE % | Final % | Rot ° | Scale | RPE10 % | First pose s | Verdict | Notes |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| TC-E1-P1 | P1 | macvo | fast | 0 | 0.38 | – | – | 0.69 | 0.029 | – | 0.0 | report | no path length (hover),G4 (rescored) |
| TC-E1-P1 | P1 | openvins | s101_replay | 0 | 0.26 | – | – | 1.63 | 0.027 | – | 3.6 | report | no path length (hover) (rescored) |
| TC-E1-P1 | P1 | openvins | s102_replay | 0 | 3.35 | – | – | 0.69 | 0.003 | – | 3.7 | report | no path length (hover) (rescored) |
| TC-E1-P1 | P1 | openvins | s7_live | 0 | 4.58 | – | – | 0.96 | 0.001 | – | 6.0 | report | no path length (hover) (rescored) |
| TC-E1-P2 | P2 | macvo | fast | 159 | 1.58 | 0.99 | 2.09 | 4.35 | 1.033 | 5.3 | 0.0 | FAIL | rot,G4 (rescored) |
| TC-E1-P2 | P2 | openvins | s101_replay | 159 | 53.21 | 33.41 | 66.37 | 10.91 | 0.311 | 183.5 | 2.2 | FAIL | diverged (rescored) |
| TC-E1-P2 | P2 | openvins | s102_replay | 133 | 28.26 | 21.26 | 70.44 | 39.57 | 0.446 | 109.4 | 19.2 | FAIL | diverged (rescored) |
| TC-E1-P2 | P2 | openvins | s7_live | 133 | 16.99 | 12.81 | 34.71 | 25.98 | 0.699 | 63.7 | 19.4 | FAIL | diverged (rescored) |
| TC-E1-P3 | P3 | macvo | fast | 500 | 3.73 | 0.75 | 1.53 | 3.26 | 1.019 | 3.5 | 0.0 | FAIL | rot,G4 (rescored) |
| TC-E1-P3 | P3 | openvins | s101_replay | 500 | 108.27 | 21.65 | 39.91 | 9.89 | 0.413 | 121.5 | 4.1 | FAIL | diverged (rescored) |
| TC-E1-P3 | P3 | openvins | s102_replay | 500 | 19.25 | 3.85 | 9.23 | 3.86 | 0.828 | 22.9 | 4.1 | FAIL | ate,drift,rot (rescored) |
| TC-E1-P3 | P3 | openvins | s7_live | 500 | 63.75 | 12.75 | 23.82 | 7.78 | 0.555 | 73.0 | 4.1 | FAIL | diverged (rescored) |
| TC-E1-P4 | P4 | macvo | fast | 607 | 1.12 | 0.18 | 0.11 | 1.38 | 0.997 | 2.6 | 0.0 | report | G4 (rescored) |
| TC-E1-P4 | P4 | openvins | s101_replay | 606 | 1.23 | 0.20 | 0.24 | 0.81 | 0.992 | 6.1 | 4.1 | report | (rescored) |
| TC-E1-P4 | P4 | openvins | s102_replay | 606 | 1.17 | 0.19 | 0.33 | 0.82 | 0.995 | 6.0 | 4.1 | report | (rescored) |
| TC-E1-P4 | P4 | openvins | s7_live | 606 | 1.07 | 0.18 | 0.26 | 0.80 | 0.990 | 4.3 | 4.1 | report | (rescored) |
| TC-E1-P5 | P5 | macvo | fast | 477 | 8.43 | 1.77 | 3.62 | 5.05 | 1.037 | 2.9 | 0.0 | report | drift,rot (rescored) |
| TC-E1-P5 | P5 | openvins | s101_replay | 456 | 160.69 | 35.23 | 104.87 | 25.45 | 0.280 | 171.8 | 14.4 | FAIL | diverged (rescored) |
| TC-E1-P5 | P5 | openvins | s102_replay | 456 | 126.15 | 27.66 | 80.73 | 24.95 | 0.341 | 135.7 | 14.4 | FAIL | diverged (rescored) |
| TC-E1-P5 | P5 | openvins | s7_live | 456 | 3195.89 | 700.70 | 1507.32 | 82.91 | 0.015 | 2765.4 | 14.4 | FAIL | diverged (rescored) |
| TC-E1-P6 | P6 | macvo | fast | 85 | 0.78 | 0.91 | 1.44 | 0.76 | 1.014 | 3.4 | 0.0 | report | (rescored) |
| TC-E1-P6 | P6 | openvins | s101_replay | 78 | 26.58 | 34.02 | 36.39 | 145.92 | 1244.613 | 100.5 | 16.2 | FAIL | diverged (rescored) |
| TC-E1-P6 | P6 | openvins | s102_replay | 78 | 26.58 | 34.02 | 36.40 | 140.19 | 1359.136 | 100.5 | 16.2 | FAIL | diverged (rescored) |
| TC-E1-P6 | P6 | openvins | s7_live | 78 | 26.59 | 34.03 | 36.41 | 154.66 | 69.957 | 100.5 | 16.2 | FAIL | diverged (rescored) |
| TC-E1-P7-low_sun | P7 | macvo | fast | 500 | 3.93 | 0.79 | 1.50 | 5.35 | 1.020 | 4.3 | 0.0 | report | rot,G4 (rescored) |
| TC-E1-P7-low_sun | P7 | openvins | s101_replay | 500 | 34.42 | 6.88 | 12.56 | 5.53 | 0.716 | 37.5 | 4.2 | report | ate,drift,rot (rescored) |
| TC-E1-P7-low_sun | P7 | openvins | s102_replay | 500 | 60.00 | 12.00 | 12.34 | 33.38 | 0.880 | 76.2 | 4.2 | FAIL | diverged (rescored) |
| TC-E1-P7-low_sun | P7 | openvins | s7_live | 500 | 40.72 | 8.14 | 16.43 | 6.96 | 0.678 | 45.2 | 4.2 | report | ate,drift,rot (rescored) |
| TC-E1-P7-overcast | P7 | macvo | fast | 500 | 2.88 | 0.58 | 1.12 | 3.04 | 1.008 | 3.4 | 0.0 | report | rot,G4 (rescored) |
| TC-E1-P7-overcast | P7 | openvins | s101_replay | 500 | 46.15 | 9.23 | 17.12 | 6.49 | 0.641 | 53.1 | 4.2 | report | ate,drift,rot (rescored) |
| TC-E1-P7-overcast | P7 | openvins | s102_replay | 500 | 35.39 | 7.08 | 13.83 | 5.69 | 0.706 | 41.5 | 4.2 | report | ate,drift,rot (rescored) |
| TC-E1-P7-overcast | P7 | openvins | s7_live | 500 | 49.87 | 9.97 | 18.89 | 7.38 | 0.623 | 57.4 | 4.1 | report | ate,drift,rot (rescored) |
| TC-E2-P1 | P1 | macvo | fast | 0 | 0.07 | – | – | 0.80 | 0.227 | – | 0.0 | report | no path length (hover) (rescored) |
| TC-E2-P1 | P1 | openvins | s101_replay | 0 | 2.74 | – | – | 0.72 | 0.002 | – | 6.1 | report | no path length (hover) (rescored) |
| TC-E2-P1 | P1 | openvins | s102_replay | 0 | 3.93 | – | – | 0.90 | 0.001 | – | 6.1 | report | no path length (hover) (rescored) |
| TC-E2-P1 | P1 | openvins | s7_live | 0 | 0.70 | – | – | 0.86 | 0.007 | – | 6.1 | report | no path length (hover) (rescored) |
| TC-E2-P2 | P2 | macvo | fast | 32 | 0.22 | 0.68 | 1.13 | 3.17 | 0.972 | 3.0 | 0.0 | FAIL | rot (rescored) |
| TC-E2-P2 | P2 | openvins | s101_replay | 32 | 3.13 | 9.74 | 2.80 | 1.77 | 0.441 | 60.9 | 4.2 | FAIL | ate,drift (rescored) |
| TC-E2-P2 | P2 | openvins | s102_replay | 32 | 3.51 | 10.93 | 4.61 | 1.75 | 0.409 | 67.2 | 4.2 | FAIL | diverged (rescored) |
| TC-E2-P2 | P2 | openvins | s7_live | 32 | 3.53 | 10.98 | 4.58 | 1.75 | 0.408 | 67.0 | 4.2 | FAIL | diverged (rescored) |
| TC-E2-P3 | P3 | macvo | fast | 70 | 0.39 | 0.56 | 1.09 | 3.96 | 0.973 | 2.5 | 0.0 | FAIL | rot (rescored) |
| TC-E2-P3 | P3 | openvins | s101_replay | 70 | 6.27 | 8.92 | 7.96 | 3.18 | 0.366 | 119.2 | 4.2 | FAIL | ate,drift,rot (rescored) |
| TC-E2-P3 | P3 | openvins | s102_replay | 70 | 6.37 | 9.06 | 7.75 | 3.06 | 0.362 | 121.5 | 4.2 | FAIL | ate,drift,rot (rescored) |
| TC-E2-P3 | P3 | openvins | s7_live | 70 | 6.35 | 9.03 | 7.04 | 3.05 | 0.362 | 121.3 | 4.1 | FAIL | ate,drift,rot (rescored) |
| TC-E1-P3-px4 | P3 | macvo | fast | 502 | 4.88 | 0.97 | 2.08 | 4.11 | 1.024 | 3.7 | 0.0 | FAIL | rot,G4 |
| TC-E1-P3-px4 | P3 | openvins | s101_replay | 502 | 75.03 | 14.96 | 17.86 | 5.55 | 20.983 | 97.1 | 4.3 | FAIL | diverged |
| TC-E1-P3-px4 | P3 | openvins | s102_replay | 502 | 75.51 | 15.05 | 17.69 | 5.88 | 23.974 | 97.5 | 4.3 | FAIL | diverged |
| TC-E1-P3-px4 | P3 | openvins | s7_live | 502 | 77.06 | 15.36 | 17.16 | 28.61 | 13.447 | 99.8 | 4.3 | FAIL | diverged |
| TC-EF-P3 | P3 | macvo | fast | 705 | 3.55 | 0.50 | 0.96 | 3.13 | 1.001 | 6.1 | 0.0 | FAIL | rot,G4 |
| TC-EF-P3 | P3 | openvins | s101_replay | 705 | 94.62 | 13.43 | 0.88 | 39.91 | 1.583 | 86.9 | 4.3 | FAIL | diverged |
| TC-EF-P3 | P3 | openvins | s102_replay | 705 | 89.53 | 12.71 | 27.90 | 10.48 | 0.562 | 70.8 | 4.3 | FAIL | diverged |
| TC-EF-P3 | P3 | openvins | s7_live | 705 | 28.16 | 4.00 | 2.25 | 3.36 | 1.301 | 26.8 | 4.3 | FAIL | ate,drift,rot |
| TC-ED-P3 | P3 | macvo | fast | 767 | 30.05 | 3.92 | 8.36 | 24.08 | 1.074 | 12.6 | 0.0 | FAIL | ate,drift,rot,scale,G4 |
| TC-ED-P3 | P3 | openvins | s101_replay | 753 | 159.05 | 21.12 | 60.21 | 34.20 | 0.372 | 127.6 | 9.8 | FAIL | diverged |
| TC-ED-P3 | P3 | openvins | s102_replay | 753 | 5152.72 | 684.16 | 1690.80 | 51.02 | 0.015 | 2904.7 | 9.8 | FAIL | diverged |
| TC-ED-P3 | P3 | openvins | s7_live | 754 | 10346.19 | 1373.04 | 2840.61 | 31.25 | 0.007 | 5142.9 | 9.8 | FAIL | diverged |
| TC-REG | REG | macvo | fast | 846 | 8.97 | 1.06 | 1.87 | 5.41 | 0.973 | 10.3 | 0.0 | report | rot,G4 (rescored) |
| TC-REG | REG | openvins | s101_replay | – | – | – | – | – | – | – | – | FAIL | no estimate |
| TC-REG | REG | openvins | s102_replay | – | – | – | – | – | – | – | – | FAIL | no estimate |
| TC-REG | REG | openvins | s7_live | 846 | 170.70 | 20.17 | 12.08 | 9.53 | 0.380 | 131.3 | 4.2 | FAIL | diverged (rescored) |

## Run-to-run spread (OpenVINS seeds)

| Case | ATEs m | std/mean | Pass |
|---|---|---|---|
| TC-E1-P1 | 4.58, 0.26, 3.35 | 0.665 | ❌ |
| TC-E1-P2 | 16.99, 53.21, 28.26 | 0.461 | ❌ |
| TC-E1-P3 | 63.75, 108.27, 19.25 | 0.570 | ❌ |
| TC-E1-P4 | 1.07, 1.23, 1.17 | 0.056 | ✅ |
| TC-E1-P5 | 3195.89, 160.69, 126.15 | 1.240 | ❌ |
| TC-E1-P6 | 26.59, 26.58, 26.58 | 0.000 | ✅ |
| TC-E1-P7-low_sun | 40.72, 34.42, 60.00 | 0.242 | ✅ |
| TC-E1-P7-overcast | 49.87, 46.15, 35.39 | 0.140 | ✅ |
| TC-E2-P1 | 0.70, 2.74, 3.93 | 0.542 | ❌ |
| TC-E2-P2 | 3.53, 3.13, 3.51 | 0.054 | ✅ |
| TC-E2-P3 | 6.35, 6.27, 6.37 | 0.007 | ✅ |
| TC-E1-P3-px4 | 77.07, 75.03, 75.51 | 0.011 | ✅ |
| TC-EF-P3 | 28.16, 94.62, 89.53 | 0.427 | ❌ |
| TC-ED-P3 | 10346.19, 159.05, 5152.72 | 0.797 | ❌ |

## Lighting (P7) vs default lighting (TC-E1-P3)

| Case | SUT | ATE m | ratio | Pass (≤ 1.5×) |
|---|---|---|---|---|
| TC-E1-P7-low_sun | openvins | 40.72 | 0.64 | ✅ |
| TC-E1-P7-low_sun | macvo | 3.93 | 1.05 | ✅ |
| TC-E1-P7-overcast | openvins | 49.87 | 0.78 | ✅ |
| TC-E1-P7-overcast | macvo | 2.88 | 0.77 | ✅ |
