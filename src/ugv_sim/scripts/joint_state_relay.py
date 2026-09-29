#!/usr/bin/env python3
"""
Joint State Relay Node — QoS bridge for Localbot simulation.

PURPOSE
───────
Gazebo bridge publishes /joint_states with BestEffort QoS.
robot_state_publisher subscribes with Reliable QoS.
These two QoS policies are INCOMPATIBLE in DDS — the subscriber receives nothing.

This node bridges the QoS mismatch:
  Subscribes:  /joint_states  (BestEffort — matches ros_gz_bridge)
  Publishes:   /joint_states_urdf  (Reliable — matches robot_state_publisher)

LOCALBOT: NO NAME REMAPPING NEEDED
───────────────────────────────────
Localbot's JointStatePublisher is configured with URDF-matching joint names:
  left_wheel_joint   (Localbot SDF)  →  left_wheel_joint   (URDF)   ✓ match
  right_wheel_joint  (Localbot SDF)  →  right_wheel_joint  (URDF)   ✓ match

The TugBot had reversed names (wheel_left_joint vs left_wheel_joint), requiring
name remapping here. Localbot eliminates that mismatch entirely.

The filter still exists for safety: if any unexpected joint names appear
(e.g., due to future model changes), they are silently dropped instead of
causing robot_state_publisher "unknown joint" warnings at 50 Hz.
Unknown joint warnings cause repeated TF updates → RViz robot model flickers white.

URDF JOINTS THAT RECEIVE JOINT STATES
──────────────────────────────────────
Only revolute/continuous joints need runtime state: left_wheel_joint, right_wheel_joint.
Fixed joints (base_joint, base_camera_joint, base_imu_joint, etc.) are published
as static TF by robot_state_publisher without needing joint state values.
Caster joints (ball type in SDF) map to fixed joints in URDF — no state needed.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import JointState


# Set of joint names that exist in the URDF as movable joints.
# Only joints in this set are forwarded to robot_state_publisher.
# All other joint names are silently dropped (future-proofing).
URDF_WHEEL_JOINTS = frozenset({
    'left_wheel_joint',
    'right_wheel_joint',
})


class JointStateRelayNode(Node):
    """
    QoS bridge: /joint_states (BestEffort) → /joint_states_urdf (Reliable).

    With Localbot, this node is a pure pass-through for the two wheel joints.
    Name remapping is not needed (Localbot joint names match URDF exactly).
    """

    def __init__(self):
        super().__init__('joint_state_relay')

        # BestEffort matches ros_gz_bridge default QoS for sensor topics.
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            durability=DurabilityPolicy.VOLATILE
        )

        # Reliable QoS matches robot_state_publisher's subscription.
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )

        self.sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._callback,
            sensor_qos
        )

        self.pub = self.create_publisher(
            JointState,
            '/joint_states_urdf',
            reliable_qos
        )

        self.get_logger().info(
            'Joint State Relay active.\n'
            '  QoS bridge: /joint_states (BestEffort) → /joint_states_urdf (Reliable)\n'
            '  Pass-through joints: left_wheel_joint, right_wheel_joint\n'
            '  (Localbot joint names already match URDF — no name remapping needed)'
        )

    def _callback(self, msg: JointState) -> None:
        """Forward wheel joints only, drop any unexpected joint names."""
        out = JointState()
        out.header = msg.header

        for i, name in enumerate(msg.name):
            if name not in URDF_WHEEL_JOINTS:
                # Drop joints unknown to the URDF (e.g., future model additions).
                # Passing unknown joints to RSP causes "unknown joint" warnings at
                # ~50 Hz, which invalidates TF and makes RViz flicker white.
                continue
            out.name.append(name)
            if i < len(msg.position):
                out.position.append(msg.position[i])
            if i < len(msg.velocity):
                out.velocity.append(msg.velocity[i])
            if i < len(msg.effort):
                out.effort.append(msg.effort[i])

        if out.name:
            self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateRelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
