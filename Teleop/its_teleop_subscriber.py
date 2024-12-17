#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial.tools.list_ports

class TeleopSubscriber(Node):
    # constructor
    def __init__(self):
        super().__init__("teleop_subscriber")
        self.subscriber_ = self.create_subscription(Twist, "its_teleoperator", self.teleop_callback, 10)
        self.get_logger().info('Initialised teleop_subscriber')
        #self.ports_ = serial.tools.list_ports.comports() 
        # initialise serial connection
        '''
        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1.0)
        except serial.SerialException:
            self.get_logger().error("Failed to open serial port")
        '''

    def teleop_callback(self, msg: Twist):
        self.get_logger().info('Recieved: v_l = ' + str(msg.linear.x) + ', v_ang = ' +  str(msg.angular.z))
        # left and right wheel angular velocities (rad/s)
        width = 0.166 # wheel width in m
        if msg.angular.z == 0:
            vl = msg.linear.x # in m/s
            vr = msg.linear.x
        else:
            vl = (2*msg.linear.x-msg.angular.z*width)/2   
            vr = (2*msg.linear.x+msg.angular.z*width)/2
        data_to_send = f'vl:{vl}, vr:{vr}\n'.encode('utf-8')
        self.ser.write(data_to_send)

def main(args=None):

    # start ros2 communication

    rclpy.init(args=args)

    # create the node

    node = TeleopSubscriber()

    # spin function runs node forever until Ctrl + C

    try:

        rclpy.spin(node)

    except:

        print("Terminating Node")

        node.ser.close()

        node.destroy_node()

        rclpy.shutdown() # Final Line

    # stop ros2 communication

    rclpy.shutdown()

if __name__ == '__main__':
    main()
