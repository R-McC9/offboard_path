#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import VehicleOdometry

class OdomListener(Node):

    def __init__(self):
        super().__init__('odom_listener')
        
        #Need custom QoS profile!!!!!
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
        print("we are in the odometery callback function")
        self.get_logger().info('Recieveing messages from odometry topic: "%s"' % msg.position)

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
