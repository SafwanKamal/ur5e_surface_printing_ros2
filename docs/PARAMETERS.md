# ROS executable parameter reference
Defaults extracted from the current source. Defaults are not physical measurements.
Use `ros2 run PACKAGE EXECUTABLE --ros-args -p NAME:=VALUE`. Some defaults are
computed expressions or installed resource paths; see the linked source.
Configuration helpers/converter use regular CLI flags (`--help`), not ROS parameters.

See [COMMANDS.md](COMMANDS.md) for effects, frames, units and workflows.

## `surface_printing execute_surface_print`

[Source](../src/surface_printing/surface_printing/execute_surface_print.py)

| Parameter | Default |
|---|---|
| `trajectory_path` | `'/tmp/surface_line_plan.yaml'` |
| `execute` | `False` |
| `extrude` | `False` |
| `simulation` | `False` |
| `hardware_confirmed` | `False` |
| `flow_ml_per_min` | `1.0` |
| `start_tolerance_rad` | `0.01` |
| `max_duration_sec` | `60.0` |
| `max_volume_ml` | `1.0` |

## `surface_printing make_demo_path`

[Source](../src/surface_printing/surface_printing/make_demo_path.py)

| Parameter | Default |
|---|---|
| `shape` | `'line'` |
| `plane` | `'xy'` |
| `frame_id` | `'world'` |
| `link_name` | `'probe_tcp'` |
| `size_mm` | `30.0` |
| `output_path` | `'/tmp/demo_path.csv'` |
| `direction` | `1.0` |

## `surface_printing make_tube_path`

[Source](../src/surface_printing/surface_printing/make_tube_path.py)

| Parameter | Default |
|---|---|
| `diameter_mm` | `10.0` |
| `height_mm` | `0.0` |
| `pitch_mm` | `0.5` |
| `step_mm` | `0.25` |
| `output_path` | `'/tmp/tube_10mm.csv'` |
| `simulation` | `False` |

## `surface_printing plan_surface_path`

[Source](../src/surface_printing/surface_printing/plan_surface_path.py)

| Parameter | Default |
|---|---|
| `csv_path` | `''` |
| `frame_id` | `''` |
| `link_name` | `''` |
| `group_name` | `'ur_manipulator'` |
| `output_path` | `'/tmp/surface_line_plan.yaml'` |
| `line_id` | `0` |
| `velocity_scale` | `0.03` |
| `acceleration_scale` | `0.03` |
| `eef_step` | `0.001` |
| `cartesian_speed_mm_s` | `2.0` |

## `syringe_controller syringe_serial_node`

[Source](../src/syringe_controller/syringe_controller/syringe_serial_node.py)

| Parameter | Default |
|---|---|
| `port` | `/dev/ttyACM0` |
| `baud_rate` | `115200` |
| `startup_timeout_sec` | `5.0` |
| `ack_timeout_sec` | `1.0` |
| `move_timeout_margin_sec` | `3.0` |
| `heartbeat_period_sec` | `0.2` |
| `maximum_step_rate_hz` | `5000.0` |
| `pulses_per_ml` | `555.555556` |
| `pulses_per_mm` | `203.006329` |
| `extrusion_direction` | `-1` |

## `ur5e_moveit_cpp add_mesh_collision_object`

[Source](../src/ur5e_moveit_cpp/src/add_mesh_collision_object.cpp)

| Parameter | Default / source expression |
|---|---|
| `object_id` | `"surface_object"` |
| `frame_id` | `"world"` |
| `mesh_resource` | `"package://ur5e_moveit_cpp/meshes/saddle/" "saddle_block_100x100.stl"` |
| `mesh_scale` | `0.001` |
| `object_x` | `0.55` |
| `object_y` | `0.00` |
| `object_z` | `0.10` |
| `object_roll_deg` | `0.0` |
| `object_pitch_deg` | `0.0` |
| `object_yaw_deg` | `0.0` |
| `service_timeout_seconds` | `10.0` |

## `ur5e_moveit_cpp check_surface_toolpath_ik`

[Source](../src/ur5e_moveit_cpp/src/check_surface_toolpath_ik.cpp)

| Parameter | Default / source expression |
|---|---|
| `csv_path` | `default_csv` |
| `planning_group` | `"ur_manipulator"` |
| `tcp_link` | `"probe_tcp"` |
| `object_x` | `0.60` |
| `object_y` | `0.00` |
| `object_z` | `0.10` |
| `object_roll_deg` | `0.0)` |
| `object_pitch_deg` | `0.0)` |
| `object_yaw_deg` | `90.0)` |
| `ik_timeout` | `0.10` |
| `max_joint_step` | `0.35` |

## `ur5e_moveit_cpp hemisphere_surface_demo`

[Source](../src/ur5e_moveit_cpp/src/hemisphere_surface_demo.cpp)

| Parameter | Default / source expression |
|---|---|
| `csv_path` | `"/home/desktop/ur5e_ws/paths/hemisphere_ring_start_00.csv"` |
| `execute` | `false` |
| `eef_step` | `0.002` |
| `jump_threshold` | `0.0` |

## `ur5e_moveit_cpp parameterized_shape_adder`

[Source](../src/ur5e_moveit_cpp/src/parameterized_shape_adder.cpp)

| Parameter | Default / source expression |
|---|---|
| `shape_type` | `"box"` |
| `object_id` | `"terminal_shape"` |
| `frame_id` | `"base_link"` |
| `center_x` | `0.50` |
| `center_y` | `0.00` |
| `center_z` | `0.30` |
| `box_x` | `0.20` |
| `box_y` | `0.20` |
| `box_z` | `0.20` |
| `radius` | `0.10` |
| `height` | `0.30` |

## `ur5e_moveit_cpp plan_execute_surface_toolpath`

[Source](../src/ur5e_moveit_cpp/src/plan_execute_surface_toolpath.cpp)

| Parameter | Default / source expression |
|---|---|
| `csv_path` | `default_csv_path` |
| `planning_group` | `"ur_manipulator"` |
| `tcp_link` | `"probe_tcp"` |
| `planning_frame` | `"world"` |
| `line_id` | `0` |
| `object_x` | `0.55` |
| `object_y` | `0.00` |
| `object_z` | `0.10` |
| `object_roll_deg` | `0.0` |
| `object_pitch_deg` | `0.0` |
| `object_yaw_deg` | `0.0` |
| `planning_time` | `15.0` |
| `planning_attempts` | `20` |
| `approach_retries` | `5` |
| `velocity_scale` | `0.10` |
| `acceleration_scale` | `0.10` |
| `eef_step` | `0.001` |
| `jump_threshold` | `0.0` |
| `minimum_fraction` | `0.999` |
| `maximum_joint_step` | `0.25` |
| `maximum_boundary_jump` | `0.02` |
| `execute` | `false` |
| `simulation` | `true` |
| `extrude` | `false` |
| `flow_ml_per_min` | `1.0` |
| `max_line_extrusion_sec` | `60.0` |
| `max_line_volume_ml` | `1.0` |
| `max_total_volume_ml` | `1.0` |
| `hardware_confirmed` | `false` |
| `extrusion_confirmed` | `false` |
| `confirm_mock_hardware` | `false` |

## `ur5e_moveit_cpp plan_surface_toolpath`

[Source](../src/ur5e_moveit_cpp/src/plan_surface_toolpath.cpp)

| Parameter | Default / source expression |
|---|---|
| `csv_path` | `default_csv_path` |
| `planning_group` | `"ur_manipulator"` |
| `tcp_link` | `"probe_tcp"` |
| `planning_frame` | `"world"` |
| `line_id` | `0` |
| `object_x` | `0.55` |
| `object_y` | `0.00` |
| `object_z` | `0.10` |
| `object_roll_deg` | `0.0` |
| `object_pitch_deg` | `0.0` |
| `object_yaw_deg` | `0.0` |
| `planning_time` | `15.0` |
| `planning_attempts` | `20` |
| `velocity_scale` | `0.10` |
| `acceleration_scale` | `0.10` |
| `eef_step` | `0.001` |
| `jump_threshold` | `0.0` |
| `minimum_fraction` | `0.999` |

## `ur5e_moveit_cpp visualize_surface_toolpath`

[Source](../src/ur5e_moveit_cpp/src/visualize_surface_toolpath.cpp)

| Parameter | Default / source expression |
|---|---|
| `csv_path` | `default_csv_path` |
| `frame_id` | `"world"` |
| `mesh_resource` | `"package://ur5e_moveit_cpp/meshes/" "apple_surface_z_up.stl"` |
| `object_x` | `0.60` |
| `object_y` | `0.00` |
| `object_z` | `0.10` |
| `object_roll_deg` | `0.0` |
| `object_pitch_deg` | `0.0` |
| `object_yaw_deg` | `90.0` |
| `mesh_scale` | `0.001` |
| `path_width` | `0.002` |
| `axis_length` | `0.025` |
| `orientation_stride` | `10` |

## `ur5e_perception object_measurement`

[Source](../src/ur5e_perception/src/object_measurement.cpp)

| Parameter | Default / source expression |
|---|---|
| `input_topic` | `"/perception/object_cloud"` |
| `center_topic` | `"/perception/object_center"` |
| `marker_topic` | `"/perception/object_bounding_box"` |

## `ur5e_perception pointcloud_preprocessor`

[Source](../src/ur5e_perception/src/pointcloud_preprocessor.cpp)

| Parameter | Default / source expression |
|---|---|
| `input_topic` | `"/camera/camera/depth/color/points"` |
| `output_topic` | `"/perception/cleaned_cloud"` |
| `min_depth` | `0.30` |
| `max_depth` | `1.00` |
| `x_min` | `-0.40` |
| `x_max` | `0.40` |
| `y_min` | `-0.30` |
| `y_max` | `0.30` |
| `voxel_size` | `0.003` |
| `mean_k` | `20` |
| `stddev_multiplier` | `1.0` |

## `ur5e_perception table_object_segmenter`

[Source](../src/ur5e_perception/src/table_object_segmenter.cpp)

| Parameter | Default / source expression |
|---|---|
| `input_topic` | `"/perception/cleaned_cloud"` |
| `table_topic` | `"/perception/table_cloud"` |
| `no_table_topic` | `"/perception/no_table_cloud"` |
| `object_topic` | `"/perception/object_cloud"` |
| `plane_distance_threshold` | `0.005` |
| `plane_max_iterations` | `200` |
| `minimum_plane_points` | `300` |
| `cluster_tolerance` | `0.015` |
| `minimum_cluster_points` | `100` |
| `maximum_cluster_points` | `100000` |
