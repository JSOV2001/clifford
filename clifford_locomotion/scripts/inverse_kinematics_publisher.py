#!/usr/bin/env python3
# ROS2 libraries
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
# Python libraries
import math
import time

class InverseKinematicsPublisher(Node):
    def __init__(self, desired_goal):
        super().__init__('inverse_kinematic_publisher')

        self.desired_goal = desired_goal
        self.max_index = len(self.desired_goal) - 1

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.to_frame = "front_left_foot" # Target frame
        self.from_frame = "front_left_base" # Source frame
    
        self.names_of_joints = [
            "front_right_shoulder_joint",
            "front_right_elbow_joint",
            "front_right_wrist_joint",
            "front_left_shoulder_joint",
            "front_left_elbow_joint",
            "front_left_wrist_joint",
            "back_right_shoulder_joint",
            "back_right_elbow_joint",
            "back_right_wrist_joint",
            "back_left_shoulder_joint",
            "back_left_elbow_joint",
            "back_left_wrist_joint"
        ]

        self.l1 = 0.05
        self.l2 = 0.12
        self.l3 = 0.14
        
        self.q1 = 0.0
        self.q2 = 0.0
        self.q3 = 0.0

        self.angle_publisher = self.create_publisher(
            JointState, 
            '/joint_states',
            10
        )

        self.goal_index = 0
        self.timer = self.create_timer(1.0, self.publish_values_in_joints)

    def get_let_transform_from_shoulder(self):
        from_frame = "front_left_shoulder"
        to_frame = "front_left_foot"
        try:
            htm = self.tf_buffer.lookup_transform(
                to_frame, # Target frame
                from_frame, # Source frame
                rclpy.time.Time().to_msg()
            )
        except TransformException:
                print(f"\tCouldn't Transform: {to_frame} from {from_frame}")
                return
        
        obtained_x = htm.transform.translation.x
        obtained_y = htm.transform.translation.y
        obtained_z = htm.transform.translation.z
        obtained_goal = [obtained_x, obtained_y, obtained_z]

        return obtained_goal
    
    def publish_values_in_joints(self):
        desired_x = self.desired_goal[self.goal_index][0]
        desired_y = self.desired_goal[self.goal_index][1]
        desired_z = self.desired_goal[self.goal_index][2]

        q_list = self.compute_inverse_kinematics_of_leg(desired_x, desired_y, desired_z)
        self.q1 = q_list[0]
        self.q2 = q_list[1]
        self.q3 = q_list[2]

        # All q values for front right leg
        front_right_q1 = 0.0
        front_right_q2 = 0.0
        front_right_q3 = 0.0

        # All q values for front left leg
        front_left_q1 = self.q1
        front_left_q2 = self.q2
        front_left_q3 = self.q3

        # All q values for back right leg
        back_right_q1 = 0.0
        back_right_q2 = 0.0
        back_right_q3 = 0.0

        # All q values for back left leg
        back_left_q1 = 0.0
        back_left_q2 = 0.0
        back_left_q3 = 0.0

        self.values_in_joints = [
            front_right_q1,
            front_right_q2, 
            front_right_q3,
            front_left_q1,
            front_left_q2, 
            front_left_q3,
            back_right_q1,
            back_right_q2,
            back_right_q3,
            back_left_q1,
            back_left_q2,
            back_left_q3
        ]

        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = self.names_of_joints
        joint_state.position = self.values_in_joints

        self.angle_publisher.publish(joint_state)
        
        try:
            htm = self.tf_buffer.lookup_transform(
                self.to_frame, # Target frame
                self.from_frame, # Source frame
                rclpy.time.Time() # Transform in current moment
            )
        except TransformException:
                print(f"\tCouldn't Transform: {self.to_frame} from {self.from_frame}")
                return
        
        obtained_x = htm.transform.translation.x
        obtained_y = htm.transform.translation.y
        obtained_z = htm.transform.translation.z

        print(f"Goal #{self.goal_index + 1}")
        print(f"\tDesired X: {desired_x * 100} cm")
        print(f"\tDesired Y: {desired_y * 100} cm")
        print(f"\tDesired Z: {desired_z * 100} cm")
        print(f"\tTheta 1: {round(math.degrees(self.q1), 2)} °")
        print(f"\tTheta 2: {round(math.degrees(self.q2), 2)} °")
        print(f"\tTheta 3: {round(math.degrees(self.q3), 2)} °")
        print(f"\tObtained Goal X: {obtained_x * 100} cm")
        print(f"\tObtained Goal Y: {obtained_y * 100} cm")
        print(f"\tObtained Goal Z: {obtained_z * 100} cm\n")

        self.max_index = len(self.desired_goal) - 1
        if (self.goal_index < self.max_index):
            self.goal_index += 1
        else:
            self.goal_index = self.max_index

    def compute_inverse_kinematics_of_leg(self, desired_x, desired_y, desired_z):
        # Compute q1
        q1 = math.atan2(desired_y, desired_z)

        # Compute q2
        a = math.sqrt((desired_x)**2 + (desired_z - self.l1)**2)

        beta = math.atan2((desired_z - self.l1), math.sqrt((desired_x**2)+(desired_z**2)))
        
        cos_alfa = ((self.l2**2)+(a**2)-(self.l3**2))/(2*self.l2*a)
        sen_alfa = math.sqrt(1-(cos_alfa)**2)
        alfa = math.atan2(sen_alfa, cos_alfa)
        
        q2 = alfa + beta - math.radians(90) 

        # Compute q3
        cos_q3 = ((self.l3**2)+(self.l2**2)-(a**2))/(2*self.l2*self.l3)
        sen_q3 = math.sqrt(1-(cos_q3)**2)
        q3 = math.atan2(sen_q3, cos_q3)

        q_list = [q1, q2, q3]
        return q_list

def main(args=None):
    rclpy.init(args=args)

    desired_goal = [    
        [0.2, 0.01, 0.01],
        [-0.2, 0.01, 0.01]
    ]

    inverse_kinematics_publisher_node = InverseKinematicsPublisher(desired_goal)
    
    try:
        rclpy.spin(inverse_kinematics_publisher_node)
    except KeyboardInterrupt:
        inverse_kinematics_publisher_node.destroy_node()
    
    rclpy.try_shutdown()

if __name__ == '__main__':
    main()