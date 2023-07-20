#!/usr/bin/env python

import numpy as np
import math
from scipy.spatial.transform import Rotation as R
import rclpy
from rclpy.clock import Clock
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint,
                          VehicleCommand, VehicleControlMode,
                          VehicleLocalPosition, VehicleStatus,
                          VehicleOdometry, VehicleAttitudeSetpoint)

class OffboardControl(Node):
    """Node for controlling vehicle in offboard mode"""

    def __init__(self) -> None:
        super().__init__('offboard_control_hex')

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create Publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.attitude_setpoint_publisher = self.create_publisher(
            VehicleAttitudeSetpoint, '/fmu/in/vehicle_attitude_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Create Subscribers
        # self.vehicle_local_position_subscriber = self.create_subscription(
        #     VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_odometry_subscriber = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', self.vehicle_odometry_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)

        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.takeoff_height = -5.0

        # Create a timer to publish commands
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.goal = [0.0, 0.0, -10.0]

        self.prev_err_x = 0.0
        self.err_sum_x = 0.0

        self.prev_err_y = 0.0
        self.err_sum_y = 0.0

        self.prev_err_z = 0.0
        self.err_sum_z = 0.0

    def vehicle_odometry_callback(self, msg):
        """Callbackfunction for vehicle_odometry topic subscriber."""
        x, y, z = msg.position
        q = [0.6586, 0.0011, -0.0085, 0.7524]

        kp = 0.7
        ki = 0.001
        kd = 30

        # Altitude controller
        err_z = self.goal[2] - z

        self.err_sum_z += err_z

        err_dif_z = err_z - self.prev_err_z

        U_z = kp * err_z + ki * self.err_sum_z + kd * err_dif_z

        # Clamp integral error term when motors are saturated
        if U_z <= -1.0:
            U_z = -1.0
            self.err_sum_z = 0

        if U_z >= 1.0:
            U_z = 1.0
            self.err_sum_z = 0

        self.prev_err_z = err_z

        # X, Y position control
        # Convert to body frame from world frame
        # Convert quaternion orientation to euler anglesn (radians)
        rpy = R.from_quat(msg.q)
        yaw_meas, pitch_meas, roll_meas = rpy.as_euler('zyx', degrees=False)

        err_x = self.goal[0] - x
        err_y = self.goal[1] - y

        # Determine position of reference in the body frame
        err_x_body, err_y_body = self.world_err_to_body_err(yaw_meas, err_x, err_y)

        self.err_sum_x += err_x_body
        self.err_sum_y += err_y_body

        err_dif_x = err_x_body - self.prev_err_x
        err_dif_y = err_y_body - self.prev_err_y

        U_x = (kp * err_x_body + ki * self.err_sum_x + kd * err_dif_x)
        U_y = (kp * err_y_body + ki * self.err_sum_y + kd * err_dif_y)

        self.prev_err_x = err_x_body
        self.prev_err_y = err_y_body

        # Clamp integral error term when motors are saturated
        if U_x <= -0.01:
            U_x = -0.01
            self.err_sum_x = 0

        if U_x >= 0.01:
            U_x = 0.01
            self.err_sum_x = 0

        self.prev_err_x = err_x

        # Clamp integral error term when motors are saturated
        if U_y <= -0.01:
            U_y = -0.01
            self.err_sum_y = 0

        if U_y >= 0.01:
            U_y = 0.01
            self.err_sum_y = 0

        self.prev_err_y = err_y

        q_d = q
        thrust_sum = math.sqrt(U_x**2 + U_y**2 + U_z**2)
        body_thrust_norm = [U_x/thrust_sum, U_y/thrust_sum, U_z/thrust_sum]
        #self.publish_attitude_setpoint(q_d, body_thrust_norm)
        self.publish_attitude_setpoint(q_d, [U_x, U_y, U_z])
        #print(body_thrust_norm)
        print(err_x, err_x_body, U_x)

    def world_err_to_body_err(self, yaw_meas, err_x, err_y):
        x_err_body = math.cos(yaw_meas)*err_x - math.sin(yaw_meas)*err_y
        y_err_body = math.sin(yaw_meas)*err_x + math.cos(yaw_meas)*err_y
        return x_err_body, y_err_body

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status

    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def land(self):
        """Switch to land mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

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

    def publish_position_setpoint(self, x: float, y: float, z: float):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 1.57079  # (90 degree)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info(f"Publishing position setpoints {[x, y, z]}")

    def publish_attitude_setpoint(self, q_d, thrust_body):
        """Publish attitude setpoint."""
        msg = VehicleAttitudeSetpoint()
        # msg.roll_body = roll_body
        # msg.pitch_body = pitch_body
        # msg.yaw_body = yaw_body
        msg.q_d = q_d #Desired quaternion orientation for quaternion based control
        msg.thrust_body = thrust_body
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

        # if self.vehicle_local_position.z > self.takeoff_height and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
        #     self.publish_position_setpoint(0.0, 0.0, self.takeoff_height)

        # elif self.vehicle_local_position.z <= self.takeoff_height:
        #     self.land()
        #     exit(0)

        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1

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