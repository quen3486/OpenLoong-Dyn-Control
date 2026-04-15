# P0 baseline (walk_mpc_wbc_v4)

- Snapshot time: 2026-04-15 14:35:38 +0800
- Command: `OPENLOONG_AUTOWALK=1 OPENLOONG_SIM_END=14 ./walk_mpc_wbc_v4` (run under `build/`)
- Binary: `build/walk_mpc_wbc_v4`
- Data files:
  - `datalog_v4_autowalk_20260415_143538.log`
  - `matlabReadDataScript_v4_autowalk_20260415_143538.txt`

## Log summary
- Rows: 14008
- Cols: 224
- Duration: 14.008 s
- `phi_range=[0.0, 0.9125]`
- `tSwing_range=[0.4, 0.4]`
- `motionState`: Stand=3010, Walk=10998
- `legState`: LSt=5651, RSt=5347, DSt=3010

## Walking-segment metrics (t>=10s)
- `js_vel_des_x` mean: 0.4000 m/s
- `baseLinVel_x` mean: 0.3501 m/s
- `vx` tracking MAE/RMSE: 0.0510 / 0.0614 m/s
- `base_rpy` RMS (roll/pitch/yaw): 1.3646 / 0.8895 / 0.7280 deg
- `|d(vx)/dt|` RMS/peak: 1.326 / 3.764 m/s^2
- peak `|dFz/dt|` (FL/FR): 5.42e5 / 5.52e5 N/s
- same-foot recurrence period: 0.716 s (1.397 Hz), equivalent cadence ~2.79 step/s
