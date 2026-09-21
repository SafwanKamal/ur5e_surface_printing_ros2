# Independent TCP speed checks

The planner reconstructs the requested TCP's Jacobian from the live MoveIt URDF and samples joint-controller spline interpolation at 101 normalized locations per segment. It supports linear, cubic and quintic interpolation according to the supplied derivative fields, rejects unsupported/missing model joints, and checks interpolated Wrist 3 samples against the configured range.

If necessary it uniformly stretches the entire trajectory to target 95% of the requested translational TCP speed limit. Timestamps scale by k, velocities by 1/k and accelerations by 1/k^2. Positions are unchanged. The executor independently repeats the speed check on the loaded plan and live model; old overspeed plans are rejected. The saved tcp_speed_check records sampled peaks and the scaling factor.

This assumes standard joint_trajectory_controller spline interpolation. Sampling with a margin is not a continuous-time proof, controller tracking guarantee or safety-rated speed limiter. It checks translational speed at the selected TCP, not speeds of every point on the tool. Physical validation and robot-side limits remain necessary. A slow plan can exceed the executor's time budget and is then rejected; do not automatically raise execution limits.

Regression using the provided 10 mm lab plan: peak sampled speed 9.513 mm/s, uniformly stretched duration 13.769 s, resulting sampled peak 1.900 mm/s for a 2 mm/s request. Test on a freshly planned simulation trajectory before hardware use.
