"""Read-only joint/TF telemetry for the dedicated domain-78 mock window."""
import json
import math
import os
import time

PREFIX = 'MOCK_STATE '


def main(args=None):
    if os.environ.get('ROS_DOMAIN_ID') != '78':
        raise SystemExit('Mock telemetry requires ROS_DOMAIN_ID=78')
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState
    from tf2_ros import Buffer, TransformListener, TransformException

    class Monitor(Node):
        def __init__(self):
            super().__init__('surface_printing_mock_monitor')
            self.joints = {}
            self.received = None
            self.joint_stamp = None
            self.buffer = Buffer()
            self.listener = TransformListener(self.buffer, self)
            self.subscription = self.create_subscription(
                JointState, '/joint_states', self.update, qos_profile_sensor_data)
            self.timer = self.create_timer(0.2, self.emit)

        def update(self, message):
            if len(message.name) != len(message.position):
                return
            if not all(math.isfinite(p) for p in message.position):
                return
            self.joints = dict(zip(message.name, message.position))
            self.received = time.monotonic()
            self.joint_stamp = message.header.stamp.sec * 1000000000 + message.header.stamp.nanosec

        def emit(self):
            joint_age = None
            if self.received is not None and self.joint_stamp:
                stamp_age = (self.get_clock().now().nanoseconds - self.joint_stamp) / 1e9
                if stamp_age >= -0.1:
                    joint_age = max(time.monotonic() - self.received, stamp_age)
            data = {'joints': self.joints, 'joint_age': joint_age,
                    'tcp': None, 'tf_age': None, 'nozzle_tilt_deg': None}
            try:
                transform = self.buffer.lookup_transform('world', 'probe_tcp', rclpy.time.Time())
                stamp = transform.header.stamp
                age = (self.get_clock().now().nanoseconds -
                       (stamp.sec * 1000000000 + stamp.nanosec)) / 1e9
                t = transform.transform.translation
                if age >= -0.1 and all(math.isfinite(v) for v in (t.x, t.y, t.z)):
                    data['tcp'] = [t.x, t.y, t.z]
                    data['tf_age'] = max(0.0, age)
                    q = transform.transform.rotation
                    norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
                    if math.isfinite(norm) and norm > 1e-9:
                        zz = 1 - 2 * ((q.x/norm)**2 + (q.y/norm)**2)
                        data['nozzle_tilt_deg'] = math.degrees(math.acos(max(-1.0, min(1.0, -zz))))
            except TransformException:
                pass
            print(PREFIX + json.dumps(data, allow_nan=False), flush=True)

    rclpy.init(args=args)
    node = Monitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
