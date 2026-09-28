#!/usr/bin/env python3
"""
Joint State Relay Node for UGV Simulation.

The Gazebo Tugbot SDF model (from Fuel) publishes joint states with names:
  - wheel_left_joint
  - wheel_right_joint
  - gripper_joint
  - gripper_hand_joint

But the UGV URDF (ugv_base.urdf.xacro) defines joints as:
  - left_wheel_joint
  - right_wheel_joint

This mismatch means robot_state_publisher never receives states for
left_wheel_joint / right_wheel_joint, so it cannot publish TF for the
wheel links, causing RViz "No transform" errors.

This node subscribes to /joint_states (from the Gazebo bridge) and
republishes with corrected joint names that match the URDF.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import JointState


# Mapping: SDF joint name (from Gazebo/Fuel model) -> URDF joint name.
# ONLY joints listed here are forwarded to robot_state_publisher.
# All other joints (gripper_joint, gripper_hand_joint, warnign_light_joint, etc.)
# are silently dropped — RSP has no URDF entries for them and would log repeated
# "unknown joint" errors at ~50 Hz, causing RViz flickering (robot blinks white).
SDF_TO_URDF_JOINT_MAP = {
    'wheel_left_joint':  'left_wheel_joint',
    'wheel_right_joint': 'right_wheel_joint',
}

# Set of URDF-known output names (used for fast O(1) lookup)
URDF_KNOWN_JOINTS = set(SDF_TO_URDF_JOINT_MAP.values())


class JointStateRelayNode(Node):
    def __init__(self):
        super().__init__('joint_state_relay')

        # Best Effort to match ros_gz_bridge sensor output
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            durability=DurabilityPolicy.VOLATILE
        )

        # Reliable QoS for robot_state_publisher (standard ROS convention)
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )

        self.sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            sensor_qos
        )

        self.pub = self.create_publisher(
            JointState,
            '/joint_states_urdf',
            reliable_qos
        )

        self.get_logger().info(
            'Joint State Relay active: mapping SDF names → URDF names\n'
            '  wheel_left_joint  → left_wheel_joint\n'
            '  wheel_right_joint → right_wheel_joint'
        )

    def joint_state_callback(self, msg: JointState):
        out = JointState()
        out.header = msg.header

        for i, name in enumerate(msg.name):
            urdf_name = SDF_TO_URDF_JOINT_MAP.get(name, None)
            # Drop joints that have no mapping to a URDF joint.
            # Gazebo publishes gripper_joint, gripper_hand_joint, warnign_light_joint,
            # etc. — none of these exist in ugv_base.urdf.xacro. Passing them to RSP
            # causes "unknown joint" warnings at 50 Hz → RViz robot blinks white.
            if urdf_name is None:
                continue
            out.name.append(urdf_name)
            if i < len(msg.position):
                out.position.append(msg.position[i])
            if i < len(msg.velocity):
                out.velocity.append(msg.velocity[i])
            if i < len(msg.effort):
                out.effort.append(msg.effort[i])

        # Only publish if we have at least one valid wheel joint
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
