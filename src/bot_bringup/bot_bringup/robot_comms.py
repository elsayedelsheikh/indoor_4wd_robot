#!/usr/bin/env python3
from math import sin, cos, pi

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

import tf_transformations
from tf2_ros import TransformBroadcaster

from nav_msgs.msg import Odometry
from bot_msgs.srv import ArduinoComm
from geometry_msgs.msg import Twist, TransformStamped, Quaternion



class RobotComms(Node):
    def __init__(self):
        super().__init__('robot_hardware_comms')

        ## Declare Parameters
        self.declare_parameter('wheel_base', 0.8382)
        self.declare_parameter('wheel_radius', 0.2286)
        self.declare_parameter('ticks_per_revolution', 12538)

        ## Get Parameters
        self.WHEEL_BASE = self.get_parameter('wheel_base').get_parameter_value().double_value
        self.WHEEL_RADIUS = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.TPR = self.get_parameter('ticks_per_revolution').get_parameter_value().integer_value

        ## Arduino Serial Client
        self.serial_cli = self.create_client(ArduinoComm, 'command_arduino', callback_group=MutuallyExclusiveCallbackGroup())
        while not self.serial_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Arduino Service not available, waiting...')

        ## Subscribers
        self.cmd_vel_sub = self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)

        ## Publishers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        ## Variables
        self.linear = 0.0
        self.angular = 0.0
        self.left_ticks = 0
        self.right_ticks = 0
        self.last_left_ticks = 0
        self.last_right_ticks = 0
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.vx =  0.0
        self.vy =  0.0
        self.vth =  0.0
        self.last_time = self.get_clock().now()

        ## Interfaces
        self.serial_req = ArduinoComm.Request()


    def cmd_vel_callback(self, msg):
        self.linear = msg.linear.x
        self.angular = msg.angular.z

    def send_request(self, linear, angular):
        # if linear > 0.0:
        #     linear = 6000.0
        # elif linear < 0.0:
        #     linear = -6000.0
        # else:
        #     linear = 0.0
        
        # if angular > 0.0:
        #     angular = 10000.0
        # elif angular < 0.0:
        #     angular = -10000.0
        # else:
        #     angular = 0.0
            
        self.serial_req.linear = linear
        self.serial_req.angular = angular

        self.future = self.serial_cli.call_async(self.serial_req)
        rclpy.spin_until_future_complete(self, self.future)
        self.data = self.future.result()

    def control_cycle(self):
        self.send_request(self.linear, self.angular)
        while rclpy.ok():
            current_time = self.get_clock().now()
            if self.data.success:
                w_,x_,y_,z_= [q for q in self.data.quaternion]
                f_l,b_l,b_r,f_r=[d for d in self.data.encoders]

                self.right_ticks = (f_r + b_r)/2
                self.left_ticks = (f_l*(-1) + b_l*(-1))/2
                delta_L = self.left_ticks - self.last_left_ticks
                delta_R = self.right_ticks - self.last_right_ticks
                dl = 2 * pi * self.WHEEL_RADIUS * delta_L / self.TPR
                dr = 2 * pi * self.WHEEL_RADIUS * delta_R / self.TPR
                dc = (dl + dr) / 2
                
                dt = (current_time.seconds_nanoseconds()[0] - self.last_time.seconds_nanoseconds()[0])
                dth = (dr-dl)/self.WHEEL_BASE
        

                if dr==dl:
                    dx=dr*cos(self.th)
                    dy=dr*sin(self.th)

                else:
                    radius=dc/dth

                    iccX=self.x-radius*sin(self.th)
                    iccY=self.y+radius*cos(self.th)

                    dx = cos(dth) * (self.x-iccX) - sin(dth) * (self.y-iccY) + iccX - self.x
                    dy = sin(dth) * (self.x-iccX) + cos(dth) * (self.y-iccY) + iccY - self.y

                self.x += dx
                self.y += dy 

                orientation_list = [x_, y_, z_, w_]
                (roll, pitch, yaw) = tf_transformations.euler_from_quaternion(orientation_list)
                self.th =yaw


                t = TransformStamped()
                t.header.stamp = current_time.to_msg()
                t.header.frame_id = 'odom'
                t.child_frame_id = 'base_link'

                t.transform.translation.x = self.x 
                t.transform.translation.y = self.y
                t.transform.translation.z = 0.0

                t.transform.rotation.x = 0.0
                t.transform.rotation.y = 0.0
                t.transform.rotation.z = z_
                t.transform.rotation.w = w_

                # Send the transformation
                self.tf_broadcaster.sendTransform(t)

                ## Publish Odometry
                odom = Odometry()
                odom.header.stamp = current_time.to_msg()
                odom.header.frame_id = "odom"

                odom.pose.pose.position.x=self.x 
                odom.pose.pose.position.y=self.y
                odom.pose.pose.position.z=0.0

                odom.pose.pose.orientation.x= 0.0
                odom.pose.pose.orientation.y=0.0
                odom.pose.pose.orientation.z=z_
                odom.pose.pose.orientation.w=w_

                if dt>0:
                    self.vx=dx/dt
                    self.vy= 0.0 
                    self.vth=dth/dt

                #set the velocity
                odom.child_frame_id = "base_link"
                odom.twist.twist.linear.x = self.vx
                odom.twist.twist.linear.y = 0.0
                odom.twist.twist.angular.z = self.vth
                self.odom_pub.publish(odom)
                self.last_time = current_time
                self.last_left_ticks = self.left_ticks
                self.last_right_ticks = self.right_ticks
            else:
                self.get_logger().info("Failed to get data")
            self.send_request(self.linear, self.angular)
            rclpy.spin_once(self)

def main():
    rclpy.init()

    robot_comms = RobotComms()
    robot_comms.control_cycle()

    robot_comms.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()