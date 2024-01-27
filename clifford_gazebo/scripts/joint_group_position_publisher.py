#!/usr/bin/env python3
# ROS2 libraries
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
# Python libraries
import math

class JointGroupPositionPublisher(Node):
    def __init__(self):
        super().__init__('joint_group_position_publisher')

        self.final_position = math.radians(20)

        # Initial position condition for Clifford's front right 
        self.front_right_shoulder_joint = math.radians(0)
        self.front_right_elbow_joint = math.radians(0)
        self.front_right_wrist_joint = math.radians(0)

        # Initial position condition for Clifford's front left 
        self.front_left_shoulder_joint = math.radians(0)
        self.front_left_elbow_joint = math.radians(0)
        self.front_left_wrist_joint = math.radians(0)

        # Initial position condition for Clifford's back right 
        self.back_right_shoulder_joint = math.radians(0)
        self.back_right_elbow_joint = math.radians(0)
        self.back_right_wrist_joint = math.radians(0)

        # Initial position condition for Clifford's back left 
        self.back_left_shoulder_joint = math.radians(0)
        self.back_left_elbow_joint = math.radians(0)
        self.back_left_wrist_joint = math.radians(0)

        self.joint_group_position_publisher = self.create_publisher(
            Float64MultiArray, 
            "/joint_group_position_controller/commands", 
            10
        )
         
        self.joint_group_position_msg = Float64MultiArray()
        self.position_timer = self.create_timer(
            0.5, 
            self.publish_position_in_joints
        )
    
    def publish_position_in_joints(self):
        self.joint_group_position_msg.data = [
            self.front_right_shoulder_joint,
            self.front_right_elbow_joint,
            self.front_right_wrist_joint,
            self.front_left_shoulder_joint,
            self.front_left_elbow_joint,
            self.front_left_wrist_joint,
            self.back_right_shoulder_joint,
            self.back_right_elbow_joint,
            self.back_right_wrist_joint,
            self.back_left_shoulder_joint,
            self.back_left_elbow_joint,
            self.back_left_wrist_joint
        ]

        self.joint_group_position_publisher.publish(self.joint_group_position_msg)
        
        print("Front Right Limb:")
        print(f"\tfront_right_shoulder_joint: {round(math.degrees(self.front_right_shoulder_joint), 2)} °")
        print(f"\tfront_right_elbow_joint: {round(math.degrees(self.front_right_elbow_joint), 2)} °")
        print(f"\tfront_right_wrist_joint: {round(math.degrees(self.front_right_wrist_joint), 2)} °") 
        print("Front Left Limb:")
        print(f"\tfront_left_shoulder_joint: {round(math.degrees(self.front_left_shoulder_joint), 2)} °")
        print(f"\tfront_left_elbow_joint: {round(math.degrees(self.front_left_elbow_joint), 2)} °")
        print(f"\tfront_left_wrist_joint: {round(math.degrees(self.front_left_wrist_joint), 2)} °") 
        print("Back Right Limb:")
        print(f"\tback_right_shoulder_joint: {round(math.degrees(self.back_right_shoulder_joint), 2)} °")
        print(f"\tback_right_elbow_joint: {round(math.degrees(self.back_right_elbow_joint), 2)} °")
        print(f"\tback_right_wrist_joint: {round(math.degrees(self.back_right_wrist_joint), 2)} °")
        print("Back Left Limb:")
        print(f"\tback_left_shoulder_joint: {round(math.degrees(self.back_left_shoulder_joint), 2)} °")
        print(f"\tback_left_elbow_joint: {round(math.degrees(self.back_left_elbow_joint), 2)} °")
        print(f"\tback_left_wrist_joint: {round(math.degrees(self.back_left_wrist_joint), 2)} °\n")

        self.front_right_shoulder_joint += math.radians(1)
        self.front_left_shoulder_joint += math.radians(1)
        self.back_right_shoulder_joint += math.radians(1)
        self.back_left_shoulder_joint += math.radians(1)

        # Final position condition for Clifford's front right leg 
        if(self.front_right_shoulder_joint >= self.final_position):
                self.front_right_shoulder_joint = self.final_position
        if(self.front_right_elbow_joint >= self.final_position):
                self.front_right_elbow_joint = self.final_position
        if(self.front_right_wrist_joint >= self.final_position):
                self.front_right_wrist_joint = self.final_position
        # Final position condition for Clifford's front left leg 
        if(self.front_left_shoulder_joint >= self.final_position):
                self.front_left_shoulder_joint = self.final_position
        if(self.front_left_elbow_joint >= self.final_position):
                self.front_left_elbow_joint = self.final_position
        if(self.front_left_wrist_joint >= self.final_position):
                self.front_left_wrist_joint = self.final_position
        # Final position condition for Clifford's back right leg 
        if(self.back_right_shoulder_joint >= self.final_position):
                self.back_right_shoulder_joint = self.final_position
        if(self.back_right_elbow_joint >= self.final_position):
                self.back_right_elbow_joint = self.final_position
        if(self.back_right_wrist_joint >= self.final_position):
                self.back_right_wrist_joint = self.final_position
        # Final position condition for Clifford's back left leg 
        if(self.back_left_shoulder_joint >= self.final_position):
                self.back_left_shoulder_joint = self.final_position
        if(self.back_left_elbow_joint >= self.final_position):
                self.back_left_elbow_joint = self.final_position
        if(self.back_left_wrist_joint >= self.final_position):
                self.back_left_wrist_joint = self.final_position

def main(args=None):
    rclpy.init(args=args)
    joint_angle_position_publisher_node = JointGroupPositionPublisher()
    try:
        rclpy.spin(joint_angle_position_publisher_node)
    except KeyboardInterrupt:
        joint_angle_position_publisher_node.destroy_node()
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()