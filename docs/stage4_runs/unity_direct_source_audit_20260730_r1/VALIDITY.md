# Invalid — performance retest required

The source-to-sink audit itself passed at its final common checkpoint
(Unity and ROS both reported IMU 60,501 and scan 2,420).  However, a single
53.100 s position-error sample reached 57.773 mm, so the maximum-error
requirement failed.  Preserve this run for diagnosis only; use the repeat
`unity_direct_source_audit_20260730_r2` as the valid source-count-audit result.
