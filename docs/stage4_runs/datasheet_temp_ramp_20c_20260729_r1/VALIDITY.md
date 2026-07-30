# Superseded — not a complete temperature-coefficient case

This run applied the ICM-42688-P accelerometer temperature terms and gyro
zero-rate temperature term, but omitted the datasheet gyro sensitivity
temperature coefficient (`0.005 %/degC`).  Its transport and estimator run was
otherwise complete; do not use its metrics as the temperature result.  The
complete, reproducible replacement is
`datasheet_temp_ramp_20c_full_20260729_r1`.
