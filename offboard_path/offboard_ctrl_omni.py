#!/usr/bin/env python

from turtle import position
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

from offboard_msgs.msg import ControlData

class OffboardControl(Node):
    """Node for controlling vehicle in offboard mode"""

    def __init__(self) -> None:
        super().__init__('offboard_control_omni')

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
        self.control_data_publisher = self.create_publisher(
            ControlData, '/ControlData', qos_profile)

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

        # Create a timer to publish commands
        self.now = 0.0
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.goal = [0.0, 0.0, 0.0]
        self.q_d = [1.0, 0.0, 0.0, 0.0]

        self.prev_err_x = 0.0
        self.err_sum_x = 0.0

        self.prev_err_y = 0.0
        self.err_sum_y = 0.0

        self.prev_err_z = 0.0
        self.err_sum_z = 0.0

    def vehicle_odometry_callback(self, msg):
        """Callbackfunction for vehicle_odometry topic subscriber."""
        x, y, z = msg.position
        w, i, j, k = msg.q
        self.t0 = msg.timestamp

        kp = 0.5
        ki = 0.001
        kd = 10

        # X, Y position control
        # Convert to body frame from world frame
        # Convert quaternion orientation to euler anglesn (radians)
        rpy = R.from_quat([i, j, k, w])
        rot_matrix = self.quat2rot(w, i, j, k)
        #roll_meas, pitch_meas, yaw_meas = rpy.as_euler('XYZ', degrees=False)

        err_x = self.goal[0] - x
        err_y = self.goal[1] - y
        err_z = self.goal[2] - z

        # Determine location of the reference position in the body frame
        err_x_body, err_y_body, err_z_body, rotMat = self.world_err_to_body_err(rot_matrix, np.array([err_x, err_y, err_z]))

        # Separate rotation matrix into lines for publishing, recombine in matlab later
        rotMat_1 = rotMat[0]
        rotMat_2 = rotMat[1]
        rotMat_3 = rotMat[2]

        # Altitude controller
        self.err_sum_z += err_z_body

        err_dif_z = err_z_body - self.prev_err_z

        U_z = kp * err_z_body + ki * self.err_sum_z + kd * err_dif_z

        # Clamp integral error term when motors are saturated
        if U_z <= -0.6:
            U_z = -0.6
            self.err_sum_z = 0

        if U_z >= 0.6:
            U_z = 0.6
            self.err_sum_z = 0

        self.prev_err_z = err_z_body

        # X,Y position controller
        self.err_sum_x += err_x_body
        self.err_sum_y += err_y_body

        err_dif_x = err_x_body - self.prev_err_x
        err_dif_y = err_y_body - self.prev_err_y

        U_x = kp * err_x_body + ki * self.err_sum_x + kd * err_dif_x
        U_y = kp * err_y_body + ki * self.err_sum_y + kd * err_dif_y

        self.prev_err_x = err_x_body
        self.prev_err_y = err_y_body

        # Clamp integral error term when motors are saturated
        if U_x <= -0.6:
            U_x = -0.6
            self.err_sum_x = 0

        if U_x >= 0.6:
            U_x = 0.6
            self.err_sum_x = 0

        # Clamp integral error term when motors are saturated
        if U_y <= -0.6:
            U_y = -0.6
            self.err_sum_y = 0

        if U_y >= 0.6:
            U_y = 0.6
            self.err_sum_y = 0

        # rot_d = R.from_euler('zxy', [0.0, 0.0, 0.0], degrees=True)
        # self.q_d = np.float32(rot_d.as_quat())
        # self.q_d = [0.5, 0.0, 0.0, 0.866025]

        # Publish control inputs
        self.publish_attitude_setpoint(self.q_d, [U_x, U_y, U_z])
        # Publish control data to different topic for graphing/debugging
        self.publish_control_data(msg.position, msg.q, rotMat_1, rotMat_2, rotMat_3, 
                                self.goal, self.q_d, [U_x, U_y, U_z],
                                [err_x_body, err_y_body, err_z_body], msg.velocity, msg.angular_velocity)

    def quat2rot(self, w, i, j, k):
        """Function for converting quaternion to rotation matrix as numpy array"""
        # Extract the values from Q
        q0 = w
        q1 = i
        q2 = j
        q3 = k
     
    # First row of the rotation matrix
        r00 = 2 * (q0 * q0 + q1 * q1) - 1
        r01 = 2 * (q1 * q2 - q0 * q3)
        r02 = 2 * (q1 * q3 + q0 * q2)
     
    # Second row of the rotation matrix
        r10 = 2 * (q1 * q2 + q0 * q3)
        r11 = 2 * (q0 * q0 + q2 * q2) - 1
        r12 = 2 * (q2 * q3 - q0 * q1)
     
    # Third row of the rotation matrix
        r20 = 2 * (q1 * q3 - q0 * q2)
        r21 = 2 * (q2 * q3 + q0 * q1)
        r22 = 2 * (q0 * q0 + q3 * q3) - 1
     
    # 3x3 rotation matrix
        rot_matrix = np.array([[r00, r01, r02],
                                [r10, r11, r12],
                                [r20, r21, r22]])

        return rot_matrix

    def world_err_to_body_err(self, rot_matrix, err_array):
        err_x_body, err_y_body, err_z_body = np.matmul(rot_matrix.transpose(), err_array)
        return err_x_body, err_y_body, err_z_body, np.float32(rot_matrix)

    def update_quaternion(self, time):
        """Updates quaternion setpoint for smooth attitude tracking."""
        if time < 8.0:
            rot_d = R.from_euler('zxy', [0.0, 0.0, 0.0])
        elif time >= 8.0 and time <= 16.0:
            rot_d = R.from_euler('zxy', [((time - 8.0)/8.0)*360.0, 0.0, 0.0], degrees=True)
        elif time >= 16.0 and time <= 24.0:
            rot_d = R.from_euler('zxy', [0.0, ((time - 16.0)/8.0)*360.0, 0.0], degrees=True)
        elif time >= 24.0 and time <= 32.0:
            rot_d = R.from_euler('zxy', [0.0, 0.0, ((time - 24.0)/8.0)*360.0], degrees=True)
        else:
            rot_d = R.from_euler('zxy', [0.0, 0.0, 0.0])
        
        self.q_d = np.float32([rot_d.as_quat()[3], rot_d.as_quat()[0], rot_d.as_quat()[1], rot_d.as_quat()[2]])
        self.q_d = np.float32(rot_d.as_quat())
        return self.q_d

    def update_goal(self, time):
        """Updates position setpoint for smooth position tracking."""
        # Fly a square with corners at (0,0), (8,0), (8,8), (0,8)
        # should take 8 seconds per side
        if time <= 8.0:
            self.goal = [0.0, 0.0, -5.0*(time/8.0)]
        # elif time >= 8.0 and time <= 16.0:
        #     self.goal = [0.0 + (time - 8.0), 0.0, -5.0]
        # elif time >= 16.0 and time <= 24.0:
        #     self.goal = [8.0, 0.0 + (time - 16.0), -5.0]
        # elif time >= 24.0 and time <= 32.0:
        #     self.goal = [8.0 - (time - 24.0), 8.0, -5.0]
        # elif time >= 32.0 and time <= 40.0:
        #     self.goal = [0.0, 8.0 - (time - 32.0), -5.0]
        else:
            self.goal = [0.0, 0.0, -5.0]
        return self.goal

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

    def publish_control_data(self, pos, q, rotMat_1, rotMat_2, rotMat_3, pos_d, q_d, thrust_body, body_errors, vel, ang_vel):
        """Publishes control data (pos/att, error, inputs, etc.) to ROS2 topic for analysis"""
        msg = ControlData()
        msg.position = pos
        msg.q = q
        msg.rotmat1 = rotMat_1
        msg.rotmat2 = rotMat_2
        msg.rotmat3 = rotMat_3
        msg.position_d = pos_d
        msg.q_d = q_d
        msg.body_errors = body_errors
        msg.velocity = vel
        msg.angular_velocity = ang_vel
        msg.body_thrust_inputs = thrust_body
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.control_data_publisher.publish(msg)

    def timer_callback(self) -> None:
        """Callback function for the timer."""
        self.publish_offboard_control_heartbeat_signal()
        self.now += 0.1
        # Uncomment this to follow a square trajctory
        self.update_goal(self.now)
        self.update_quaternion(self.now)

        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()

        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1

        # if self.start_rotate == True:
        #     self.now = 0
        #     #self.update_quaternion(self.now)

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