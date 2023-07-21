#!/usr/bin/env python
import numpy as np
import math
from scipy.spatial.transform import Rotation as R

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

        self.goal = [1.0, 1.0, -5]

    def odometry_callback(self, msg):
        x, y, z = msg.position
        w, i, j, k = msg.q
        rpy = R.from_quat([i, j, k, w])
        # roll_meas, pitch_meas, yaw_meas = rpy.as_euler('XYZ', degrees=False)
        # print(roll_meas, pitch_meas, yaw_meas)
        print(msg.q)

        # Error in the world frame (Orientation does not matter here)
        err_x = self.goal[0] - x
        err_y = self.goal[1] - y
        err_z = self.goal[2] - z

        # Error in the body frame (Orientation does matter)
        # Use rotation matrix from quaternion orientation to convert to body frame
        err_x_body, err_y_body, err_z_body = self.world_err_to_body_err(rpy, np.array([err_x, err_y, err_z]))

        #print(err_x_body, err_y_body, err_z_body)

    def world_err_to_body_err(self, rpy, err_array):
        err_x_body, err_y_body, err_z_body = np.dot(rpy.as_matrix(), err_array)
        return err_x_body, err_y_body, err_z_body

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
