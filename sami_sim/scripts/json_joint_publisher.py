#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time
import threading
import json
import math
import os

from ament_index_python.packages import get_package_share_directory

behavior = 'TEST.json' # Change this to the desired behavior file

class RobotJointPublisher(Node):
    def __init__(self):
        super().__init__('robot_joint_publisher')
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)

        # Define all joints in the robot from XACRO
        self.joint_positions = {
            "RightHip": 0.0, "TorsoBow": 0.0, "TorsoTilt": 0.0, "RightChest": 0.0,
            "HeadTurn": 0.0, "HeadNod": 0.0, "RightBicep": 0.0, "RightElbow": 0.0,
            "RightKnee": 0.0, "RightAnkle": 0.0, "HeadTilt": 0.0, "LeftHip": 0.0,
            "LeftKnee": 0.0, "LeftAnkle": 0.0, "LeftChest": 0.0, "LeftShoulder": 0.0,
            "LeftBicep": 0.0, "LeftElbow": 0.0, "RightShoulder": 0.0
        }

        # Joint direction multipliers (1 = normal, -1 = inverted)
        self.joint_directions = {
            "RightChest": 1,
            "RightShoulder": -1,
            "RightBicep": -1,
            "RightElbow": 1,
            "RightKnee": 1,
            "RightAnkle": -1,
            "LeftChest": -1,
            "LeftShoulder": -1,
            "LeftBicep": 1,
            "LeftElbow": -1,
            "LeftKnee": 1,
            "LeftAnkle": 1,
            "TorsoBow": 1,
            "TorsoTilt": 1,
            "HeadTurn": 1,
            "HeadNod": 1,
            "HeadTilt": 1,
            "RightHip": -1,
            "LeftHip": 1,
        }

        # Home offsets (in degrees) to align JSON with model zero positions
        self.joint_offsets = {
            "LeftChest": -115.0,
            "LeftShoulder": -180.0,
            "LeftBicep": -115.0,
            "LeftElbow": -105.0,
            "RightChest": -135.0,
            "RightShoulder": -85.0,
            "RightBicep": -115.0,
            "RightElbow": -90.0,
            "LeftHip": -96.0,
            "LeftKnee": -80.0,
            "LeftAnkle": -90.0,
            "RightHip": -84.0,
            "RightKnee": -90.0,
            "RightAnkle": -80.0,
            "HeadTurn": -120.0,
            "HeadNod": -125.0,
            "HeadTilt": -125.0,
            "TorsoBow": -125.0,
            "TorsoTilt": -115.0
        }

        self.timer = self.create_timer(0.1, self.publish_joint_states)
        self.get_logger().info("Robot joint publisher started")

    def publish_joint_states(self):
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = list(self.joint_positions.keys())
        joint_state_msg.position = list(self.joint_positions.values())
        self.publisher.publish(joint_state_msg)

    def move_joint(self, joint_name, target_angle_degrees, duration=2.0):
        if joint_name not in self.joint_positions:
            self.get_logger().error(f"Joint {joint_name} not found in robot model")
            return

        direction = self.joint_directions.get(joint_name, 1)
        offset = self.joint_offsets.get(joint_name, 0.0)
        target_position = math.radians((target_angle_degrees + offset) * direction)

        start_position = self.joint_positions[joint_name]
        start_time = time.time()

        def update_position():
            while time.time() - start_time < duration:
                elapsed = time.time() - start_time
                progress = min(elapsed / duration, 1.0)
                new_position = start_position + progress * (target_position - start_position)
                self.joint_positions[joint_name] = new_position

                if int(elapsed * 10) % 5 == 0:
                    angle_degrees = math.degrees(new_position) * direction
                    self.get_logger().info(f"Moving {joint_name} to {angle_degrees:.1f} degrees")

                time.sleep(0.01)

            self.joint_positions[joint_name] = target_position
            self.get_logger().info(f"Final position of {joint_name} reached: {target_angle_degrees} degrees")

        thread = threading.Thread(target=update_position, daemon=True)
        thread.start()
        return thread

    def move_joints_simultaneously(self, joint_angles, duration=2.0):
        threads = []
        for joint_info in joint_angles:
            joint_name = joint_info["Joint"]
            angle = joint_info["Angle"]
            thread = self.move_joint(joint_name, angle, duration)
            threads.append(thread)

        for thread in threads:
            thread.join()

    def execute_animation(self, animation_file):
        try:
            with open(animation_file, 'r') as f:
                animation_data = json.load(f)

            self.get_logger().info(f"Executing animation: {animation_data['Name']}")

            for i, keyframe in enumerate(animation_data['Keyframes']):
                self.get_logger().info(f"Executing keyframe {i+1}/{len(animation_data['Keyframes'])}")

                if keyframe.get("HasEmote") == "True" and "Expression" in keyframe:
                    self.get_logger().info(f"Expression: {keyframe['Expression']}")

                if keyframe.get("HasJoints") == "True" and "JointAngles" in keyframe:
                    move_time = float(keyframe.get("JointMoveTime", 1))
                    self.move_joints_simultaneously(keyframe["JointAngles"], move_time)

                wait_time = float(keyframe.get("WaitTime", 0)) / 1000.0
                self.get_logger().info(f"Waiting for {wait_time} seconds")
                time.sleep(wait_time)

            self.get_logger().info(f"Animation {animation_data['Name']} completed")

        except Exception as e:
            self.get_logger().error(f"Error executing animation: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    publisher = RobotJointPublisher()

    package_name = 'sami_sim'
    animation_file = os.path.join(
        get_package_share_directory(package_name),
        'behaviors',
        behavior
    )

    animation_thread = threading.Thread(
        target=publisher.execute_animation,
        args=(animation_file,),
        daemon=True
    )
    animation_thread.start()

    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass

    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
