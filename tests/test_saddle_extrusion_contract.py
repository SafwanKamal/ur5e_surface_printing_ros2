import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SOURCE = (
    ROOT
    / "src/ur5e_moveit_cpp/src/plan_execute_surface_toolpath.cpp"
).read_text()
CMAKE = (ROOT / "src/ur5e_moveit_cpp/CMakeLists.txt").read_text()
PACKAGE = (ROOT / "src/ur5e_moveit_cpp/package.xml").read_text()


class SaddleExtrusionContractTests(unittest.TestCase):
    def test_physical_and_extrusion_confirmations_are_required(self):
        self.assertIn('"hardware_confirmed"', SOURCE)
        self.assertIn('"extrusion_confirmed"', SOURCE)
        self.assertIn("Extrusion requires physical execution", SOURCE)

    def test_only_surface_segments_request_extrusion(self):
        self.assertIn("!is_transition && extrude", SOURCE)
        self.assertIn("establishStopped();", SOURCE)
        self.assertNotIn("move_group.execute(segment)", SOURCE)

    def test_motion_is_observed_before_flow_starts(self):
        motion_check = SOURCE.index(
            'tripFault("No measured robot motion before extrusion")'
        )
        flow_start = SOURCE.index("startFlow(requested_flow)")
        self.assertLess(motion_check, flow_start)

    def test_fault_path_stops_pump_and_cancels_robot(self):
        fault_method = SOURCE[SOURCE.index("void tripFault("):]
        self.assertIn("stop_client_->async_send_request", fault_method)
        self.assertIn("action_client_->async_cancel_goal", fault_method)

    def test_volume_and_duration_budgets_exist(self):
        for parameter in (
            "max_line_extrusion_sec",
            "max_line_volume_ml",
            "max_total_volume_ml",
        ):
            self.assertIn(f'"{parameter}"', SOURCE)

    def test_approaches_use_collision_free_ik_joint_goals(self):
        self.assertIn('GetPositionIK>("/compute_ik")', SOURCE)
        self.assertIn("avoid_collisions = true", SOURCE)
        self.assertIn("setJointValueTarget(joint_targets)", SOURCE)
        self.assertNotIn(
            "setPoseTarget(approach_target,tcp_link)",
            SOURCE,
        )
        self.assertNotIn("getCurrentState(10.0)", SOURCE)

    def test_build_dependencies_are_declared(self):
        for dependency in (
            "rclcpp_action",
            "std_msgs",
            "std_srvs",
            "syringe_interfaces",
        ):
            self.assertIn(dependency, CMAKE)
            self.assertIn(dependency, PACKAGE)


if __name__ == "__main__":
    unittest.main()
