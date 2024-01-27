#!/usr/bin/env python3
# ROS2 libraries
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
# Python libraries
import serial
import math

class SerialPortWriter(Node):
    def __init__(self):
        super().__init__('serial_port_writer')

        self.position_subscriber = self.create_subscription(
            JointState, 
            '/joint_states', 
            self.joint_states_callback, 
            10
        )

        # Serial port's information
        self.microcontroller_port = "/dev/ttyUSB0"
        self.microcontroller_baudrate = 1000000
        self.serial_port = serial.Serial(
            self.microcontroller_port, 
            self.microcontroller_baudrate, 
            timeout=10
        )
        self.mapped_msg = String()

    def map_position_into_bits(self, value, in_min, in_max, out_min, out_max):
        mapped_data_list = (value - in_min) * (out_max - out_min) / ((in_max - in_min) + out_min)
        return mapped_data_list
    
    def write_into_serial_port(self, mapped_data_list):
        self.mapped_msg.data = str(mapped_data_list)
        encoded_mapped_msg = self.mapped_msg.data.encode()
        data_out = f"{encoded_mapped_msg};"
        print(data_out)
        self.serial_port.write(bytes(data_out, 'utf-8'))
        
    def joint_states_callback(self, joint_states_msg):
        print("JointState received!")

        front_right_q1_in_degrees = round(math.degrees(joint_states_msg.position[6]), 2)
        front_right_q2_in_degrees = round(math.degrees(joint_states_msg.position[7]), 2)
        front_right_q3_in_degrees = round(math.degrees(joint_states_msg.position[8]), 2)
        front_left_q1_in_degrees = round(math.degrees(joint_states_msg.position[0]), 2)
        front_left_q2_in_degrees = round(math.degrees(joint_states_msg.position[1]), 2)
        front_left_q3_in_degrees = round(math.degrees(joint_states_msg.position[2]), 2)
        back_right_q1_in_degrees = round(math.degrees(joint_states_msg.position[9]), 2)
        back_right_q2_in_degrees = round(math.degrees(joint_states_msg.position[10]), 2)
        back_right_q3_in_degrees = round(math.degrees(joint_states_msg.position[11]), 2)
        back_left_q1_in_degrees = round(math.degrees(joint_states_msg.position[3]), 2)
        back_left_q2_in_degrees = round(math.degrees(joint_states_msg.position[4]), 2)
        back_left_q3_in_degrees = round(math.degrees(joint_states_msg.position[5]), 2)

        front_right_q1_in_bits = self.map_position_into_bits(front_right_q1_in_degrees, -150, 150, 0, 1023)
        front_right_q2_in_bits = self.map_position_into_bits(front_right_q2_in_degrees, -150, 150, 0, 1023)
        front_right_q3_in_bits = self.map_position_into_bits(front_right_q3_in_degrees, -150, 150, 0, 1023)

        front_left_q1_in_bits = self.map_position_into_bits(front_left_q1_in_degrees, -150,150, 0, 1023)
        front_left_q2_in_bits = self.map_position_into_bits(front_left_q2_in_degrees, -150,150, 0, 1023)
        front_left_q3_in_bits = self.map_position_into_bits(front_left_q3_in_degrees, -150, 150, 0, 1023)
        
        back_right_q1_in_bits = self.map_position_into_bits(back_right_q1_in_degrees, -150, 150, 0, 1023)
        back_right_q2_in_bits = self.map_position_into_bits(back_right_q2_in_degrees, -150, 150, 0, 1023)
        back_right_q3_in_bits = self.map_position_into_bits(back_right_q3_in_degrees, -150, 150, 0, 1023)

        back_left_q1_in_bits = self.map_position_into_bits(back_left_q1_in_degrees, -150, 150, 0, 1023)
        back_left_q2_in_bits = self.map_position_into_bits(back_left_q2_in_degrees, -150, 150, 0, 1023)
        back_left_q3_in_bits = self.map_position_into_bits(back_left_q3_in_degrees, -150, 150, 0, 1023)

        mapped_data_list = [
            front_left_q1_in_bits,
            front_left_q2_in_bits,
            front_left_q3_in_bits,
            back_left_q1_in_bits,
            back_left_q2_in_bits,
            back_left_q3_in_bits
        ]

        self.write_into_serial_port(mapped_data_list)

def main(args=None):
    rclpy.init(args=args)
    serial_port_writer_node = SerialPortWriter()
    try:
        rclpy.spin(serial_port_writer_node)
    except KeyboardInterrupt:
        serial_port_writer_node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()