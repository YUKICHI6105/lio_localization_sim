# Invalid — Unity Editor did not attach

ROS 2 startup completed and the TCP endpoint on port 10000 reached `READY`.
No Unity TCP client attached, so `/imu/data`, `/scan`, and
`/ground_truth_pose` were never published and the evaluator never started its
measurement window.  This run is preserved only as a connection-attempt log;
it is not a localisation or closed-loop-control result.

The Windows Unity MCP relay also returned no tool list, indicating that no
controllable Editor session was attached to its named pipe.  Retry only after
the Windows Editor has been opened with `Robocon2026Sim` and its MCP relay has
registered the Editor session.
