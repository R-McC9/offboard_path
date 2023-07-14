#!/usr/bin/env python
import numpy as np
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import VehicleOdometry

class OdomListener(Node):

    def __init__(self):
        super().__init__('odom_listener')
        
        #Need custom QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        #create subscriber
        self.odometry_listener_ = self.create_subscription(
            VehicleOdometry, 
            '/fmu/out/vehicle_odometry', 
            self.odometry_callback,
            qos_profile)
        self.odometry_listener_

    def odometry_callback(self, msg):
        w, i, j, k = msg.q
        roll_meas, pitch_meas, yaw_meas = self.euler_from_quaternion(w, i, j, k)
        print(yaw_meas)


    def euler_from_quaternion(self, w, i, j, k):
        jsqr = j * j
       
        t0 = +2.0 * (w * i + j * k)
        t1 = +1.0 - 2.0 * (i * i + jsqr)
        roll_x = np.degrees(np.arctan2(t0, t1))
        
        t2 = +2.0 * (w * j - k * i)
        # t2 = +1.0 if t2 > +1.0 else t2
        # t2 = -1.0 if t2 < -1.0 else t2
        t2 = np.where(t2>+1.0, +1.0, t2)
        t2 = np.where(t2<-1.0, -1.0, t2)
        pitch_y = np.degrees(np.arcsin(t2))
        
        t3 = +2.0 * (w * k + i * j)
        t4 = +1.0 - 2.0 * (jsqr + k * k)
        yaw_z = np.degrees(np.arctan2(t3, t4))
        
        return roll_x, pitch_y, yaw_z # in radians

    def world_err_to_body_err(self, yaw_meas, err_x, err_y):
        x_err_body = math.cos(yaw_meas)*err_x - math.sin(yaw_meas)*err_y
        y_err_body = math.sin(yaw_meas)*err_x + math.cos(yaw_meas)*err_y
        return x_err_body, y_err_body

def main(args=None):
    rclpy.init(args=args)
    odom_listener = OdomListener()

    try:
        odom_listener.get_logger().info('Starting odometry listener node, shut down with CTRL-C')
        rclpy.spin(odom_listener)
    except KeyboardInterrupt:
        odom_listener.get_logger().info('Keyboard interrupt, shutting down.\n')
    #Destroy node explicitly on keyboard interrupt
    odom_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
