#! /usr/bin/env python

import numpy as np
from time import sleep
import math
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.clock import Clock
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import (OffboardControlMode, VehicleAttitudeSetpoint, VehicleCommand,
                          VehicleOdometry, VehicleStatus)

class OffboardControl(Node):
    """Node for doing offboard control"""

    def __init__(self) -> None:
        super().__init__('offboard_ctrl_quad_attitude')

        #Configure QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create Publishers
        self.attitude_setpoint_publisher = self.create_publisher(
            VehicleAttitudeSetpoint, '/fmu/in/vehicle_attitude_setpoint', qos_profile)
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Create subscribers
        self.vehicle_odometry_subscriber = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', self.vehicle_odometry_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)

        # Initialize Variables
        self.offboard_setpoint_counter = 0
        self.vehicle_status = VehicleStatus()

        # Create timer for command publication
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Establish goal position
        self.goal = [0.0, 0.0, -1.0]

        # Initialize controller errors
        self.prev_err_x = 0.0
        self.err_sum_x = 0.0

        self.prev_err_y = 0.0
        self.err_sum_y = 0.0

        self.prev_err_z = 0.0
        self.err_sum_z = 0.0

    def vehicle_odometry_callback(self, msg):
        """Callback function for vehicle_odometry topic subscription"""
        # Compute controller inputs every time this function runs
        x, y, z = msg.position
        w, i, j, k = msg.q

        kpx = 0.7
        kix = 0.001
        kdx = 30

        kpy = 0.7
        kiy = 0.001
        kdy = 30

        kpz = 0.7
        kiz = 0.001
        kdz = 30

        err_x = self.goal[0] - x
        err_y = self.goal[1] - y
        err_z = self.goal[2] - z

        rpy = R.from_quat([i, j, k, w])
        
        err_x_body, err_y_body, err_z_body = self.world_err_to_body_err(rpy, np.array([err_x, err_y, err_z]))

        # Altitude controller
        self.err_sum_z += err_z_body

        err_dif_z = err_z_body - self.prev_err_z

        U_z = kpz * err_z_body + kiz * self.err_sum_z + kdz * err_dif_z

        # Clamp integral error term when motors are saturated
        if U_z <= -0.3:
            U_z = -0.3
            self.err_sum_z = 0

        if U_z >= 0.3:
            U_z = 0.3
            self.err_sum_z = 0

        self.prev_err_z = err_z_body

        # Position controller
        # Compute error between desired position and actual position in the body frame
        # From error determine a direction and magnitude to roll/pitch to
        # Convert desired pitch/roll into a quaternion with scipy, give desired quaternion to attitude setpoint
        self.err_sum_x += err_x_body
        self.err_sum_y += err_y_body

        err_dif_x = err_x_body - self.prev_err_x
        err_dif_y = err_y_body - self.prev_err_y

        roll = kpx * err_x_body + kix * self.err_sum_x + kdx * err_dif_x
        pitch = kpy * err_y_body + kiy * self.err_sum_y + kdy * err_dif_y

        # Limit how far the drone can actually pitch/roll
        # rpy_d = R.from_euler('xyz', roll_d, pitch_d, yaw_d)

        # q_d = rpy_d.R.as_quat()

        q_d = [1.0, 0.0, 0.0, 0.0]

        self.publish_attitiude_setpoint(q_d, U_z)
        print(z, err_z, U_z)

    def world_err_to_body_err(self, rpy, err_array):
        err_x_body, err_y_body, err_z_body = np.matmul(rpy.as_matrix(), err_array)
        return err_x_body, err_y_body, err_z_body

    def arm(self):
        """Send an arm command to the vehicle"""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info("Arm command sent")

    def disarm(self):
        """Send a disarm command to the vehicle"""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info("Disarm command sent")

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def land(self):
        """Send a land commadn to the vehicle"""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_NAV_LAND, param1=21.0)
        self.get_logger().info("Landing...")
        sleep(3)

    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = True
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_attitiude_setpoint(self, q_d, U_z):
        """Publish attitude setpoint"""
        msg = VehicleAttitudeSetpoint()
        # msg.roll_body = roll_d
        # msg.pitch_body = pitch_d
        # msg.yaw_body = yaw_d
        msg.q_d = q_d
        msg.thrust_body = [0.0, 0.0, U_z]
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.attitude_setpoint_publisher.publish(msg)

    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def timer_callback(self) -> None:
        """Callback function for the timer."""
        self.publish_offboard_control_heartbeat_signal()

        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()

        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status
        
def main(args=None) -> None:
    print('Starting offboard control node...')
    rclpy.init(args=args)
    try:
        offboard_control = OffboardControl()
        rclpy.spin(offboard_control)
    except KeyboardInterrupt:
        offboard_control.get_logger().info('Keyboard interrupt, shutting down.\n')

    offboard_control.land()
    offboard_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)