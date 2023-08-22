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

from offboard_msgs.msg import ControlData

from geometry_msgs.msg import WrenchStamped

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

        # Gazebo quality of service profile
        gazebo_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
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
        # Subscriber for force/torque feedback
        self.force_torque_subscriber = self.create_subscription(
            WrenchStamped, '/wrench', self.force_torque_callback, gazebo_qos)

        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()

        # Create a timer to publish commands
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Create time for faster trajectory updating
        self.now = 0.0
        self.traj_timer = self.create_timer(0.01, self.traj_timer_callback)

        self.goal = [0.0, 0.0, 0.0]
        self.start_pos = [0.0, 0.0, 0.0]
        self.q_d = [1.0, 0.0, 0.0, 0.0]

        self.prev_err_x = 0.0
        self.err_sum_x = 0.0

        self.prev_err_y = 0.0
        self.err_sum_y = 0.0

        self.prev_err_z = 0.0
        self.err_sum_z = 0.0

        self.count = 0
        self.num = 0
        self.prev_U = np.empty((20,3))

    def vehicle_odometry_callback(self, msg):
        """Callbackfunction for vehicle_odometry topic subscriber."""
        x, y, z = msg.position
        w, i, j, k = msg.q

        kpz = 0.6
        kiz = 0.006
        kdz = 20

        kpx = 0.7
        kix = 0.0002
        kdx = 30

        kpy = 0.7
        kiy = 0.0002
        kdy = 30

        # X, Y position control
        # Calculate rotation matrix from quaternion
        # rot_matrix = self.quat2rot(w, i, j, k)
        rpy = R.from_quat([i, j, k, w])

        err_x = self.goal[0] - x
        err_y = self.goal[1] - y
        err_z = self.goal[2] - z

        # Determine position of reference in the body frame
        err_x_body, err_y_body, err_z_body = self.world_err_to_body_err(rpy, np.array([err_x, err_y, err_z]))

        # Altitude controller
        self.err_sum_z += err_z_body

        err_dif_z = err_z_body - self.prev_err_z

        U_z = kpz * err_z_body + kiz * self.err_sum_z + kdz * err_dif_z

        U_z_max = -1.0

        U_x_max = -0.3
        U_y_max = -0.3

        # Clamp integral error term when motors are saturated
        if U_z <= U_z_max or U_z >= 0.0:
            U_z = np.clip(U_z, -1.0, 0.0)
            self.err_sum_z = 0

        # apply minimum throttle value to prevent chattering at takeoff
        if U_z >= -0.1 and U_z <= 0.0:
            U_z = -0.1

        self.prev_err_z = err_z_body

        # X,Y Position Controller
        self.err_sum_x += err_x_body
        self.err_sum_y += err_y_body

        err_dif_x = err_x_body - self.prev_err_x
        err_dif_y = err_y_body - self.prev_err_y

        U_x = (kpx * err_x_body + kix * self.err_sum_x + kdx * err_dif_x)
        U_y = (kpy * err_y_body + kiy * self.err_sum_y + kdy * err_dif_y)

        self.prev_err_x = err_x_body
        self.prev_err_y = err_y_body

        # Clamp integral error term when motors are saturated
        if U_x <= U_x_max or U_x >= -U_x_max:
            U_x = np.clip(U_x, U_x_max, -U_x_max)
            self.err_sum_x = 0

        # Clamp integral error term when motors are saturated
        if U_y <= U_y_max or U_y >= -U_y_max:
            U_y = np.clip(U_y, U_y_max, -U_y_max)
            self.err_sum_y = 0

        # U_z = (U_z + np.sum(self.prev_U, axis=0)[2])/21

        # rot_d = R.from_euler('zxy', [0.0, 0.0, 0.0], degrees=True)
        # rot_d = rot_d.as_quat()
        # self.q_d = np.float32([rot_d[3], rot_d[0], rot_d[1], rot_d[2]])
        # self.q_d = [1.0, 0.0, 0.0, 0.0]
        # self.goal = [0.0, 0.0, -0.6]
        self.publish_attitude_setpoint(self.q_d, [U_x, U_y, U_z])

        self.save_U([U_x, U_y, U_z])

        self.publish_control_data(msg.position, msg.q, self.goal, self.q_d, [err_x_body, err_y_body, err_z_body],
                               [U_x, U_y, U_z], msg.velocity, msg.angular_velocity)

    def save_U(self, U):
        """Saves N + 1 previous inputs for averaging
        U = [U_x, U_y, U_z]
        """
        N = 20
        # Write previous input to Nx3 array
        if self.count < N:
            self.prev_U[self.count] = U
            self.count += 1
            return
        elif self.count >= N:
            self.prev_U[self.count - N] = U
            self.count = 0
            return

    def force_torque_callback(self, msg):
        """Callback function for force/torque sensor"""
        # self.current_F = msg.wrench.force
        # return self.current_F, self.current_T

    def update_quaternion(self, time):
        """Updates quaternion setpoint for smooth attitude tracking."""
        # Hold attitude @ 0 pitch/roll/yaw
        rot_d = R.from_euler('zxy', [0.0, 0.0, 0.0])

        #Pitch +-5 degrees over 30 seconds, hold at 0 rotation at the end
        # if time < 10.0:
        #     rot_d = R.from_euler('zxy', [0.0, 0.0, 0.0])
        # elif time >= 10.0 and time <= 40.0:
        #     rot_d = R.from_euler('zxy', [0.0, 0.0, np.sin(4*math.pi*((time - 10.0)/30.0))*5.0], degrees=True)
        # else:
        #     rot_d = R.from_euler('zxy', [0.0, 0.0, 0.0], degrees=True)

        #Roll +-5 degrees over 30 seconds, hold at 0 rotation after
        # if time < 10.0:
        #     rot_d = R.from_euler('zxy', [0.0, 0.0, 0.0])
        # elif time >= 10.0 and time <= 40.0:
        #     rot_d = R.from_euler('zxy', [0.0, np.sin(4*math.pi*((time - 10.0)/30.0))*5.0, 0.0], degrees=True)
        # else:
        #     rot_d = R.from_euler('zxy', [0.0, 0.0, 0.0], degrees=True)
        
        self.q_d = np.float32([rot_d.as_quat()[3], rot_d.as_quat()[0], rot_d.as_quat()[1], rot_d.as_quat()[2]])
        return self.q_d

    def update_goal(self, time):
        """Updates position setpoint for smooth position tracking."""
        # Slowly ascend to 0.8m over 8 seconds and hold
        if time <= 8.0:
            self.goal = np.float32([0.0, 0.0, -0.8*(time/8.0)])
        else:
            self.goal = [0.0, 0.0, -0.8]

        # Slowly ascend to 0.7m, slide left and right, hold @ origin
        # if time <= 8.0:
        #     self.goal = np.float32([0.0, 0.0, -0.7*(time/8.0)])
        # elif time > 8.0 and time <= 12.0:
        #     self.goal = np.float32([0.0, 1.0*((time - 8.0)/4.0), -0.7])
        # elif time > 12.0 and time <= 20.0:
        #     self.goal = np.float32([0.0, 1.0 - 2.0*((time - 12.0)/8.0), -0.7])
        # elif time > 20.0 and time <= 24.0:
        #     self.goal = np.float32([0.0, -1.0 + 1.0*((time - 20.0)/4.0), -0.7])
        # else:
        #     self.goal = [0.0, 0.0, -0.7]

        # Slowly ascend to 0.7m, slide forward and backward, hold @ origin
        # if time <= 8.0:
        #     self.goal = np.float32([0.0, 0.0, -0.7*(time/8.0)])
        # elif time > 8.0 and time <= 12.0:
        #     self.goal = np.float32([1.0*((time - 8.0)/4.0), 0.0, -0.7])
        # elif time > 12.0 and time <= 20.0:
        #     self.goal = np.float32([1.0 - 2.0*((time - 12.0)/8.0), 0.0, -0.7])
        # elif time > 20.0 and time <= 24.0:
        #     self.goal = np.float32([-1.0 + 1.0*((time - 20.0)/4.0), 0.0, -0.7])
        # else:
        #     self.goal = [0.0, 0.0, -0.7]

        # Slowly ascend to 0.5m, slide around a 1m square
        # if time <= 8.0:
        #     self.goal = np.float32([0.0, 0.0, -0.7*(time/8.0)])
        # elif time > 8.0 and time <= 12.0:
        #     self.goal = np.float32([0.0, -1.0*((time - 8.0)/4.0), -0.7])
        # elif time > 12.0 and time <= 16.0:
        #     self.goal = np.float32([-1.0*((time - 12.0)/4.0), -1.0, -0.7])
        # elif time > 16.0 and time <= 20.0:
        #     self.goal = np.float32([-1.0, -1.0 + 1.0*((time - 16.0)/4.0), -0.7])
        # elif time > 20.0 and time <= 24.0:
        #     self.goal = np.float32([-1.0 + 1.0*((time - 20.0)/4.0), 0.0, -0.7])
        # else:
        #     self.goal = [0.0, 0.0, -0.7]

        # Slowly ascend to 0.5m, trace a horizontal figure eight
        # if time <= 8.0:
        #     self.goal = np.float32([0.0, 0.0, -1.0*(time/8.0)])
        # elif time > 8.0 and time <= 60.0:
        #     self.goal = np.float32([3.0*(np.sin(2*np.pi*((time - 8.0)/52.0))/(1 + np.sin(2*np.pi*((time - 8.0)/52.0)))**2), 
        #                             3.0*(np.sin(2*np.pi*((time - 8.0)/52.0))*np.cos(2*np.pi*((time - 8.0)/52.0)))/(1 + np.sin(2*np.pi*((time - 8.0)/52.0))**2), 
        #                             -1.0])
        # else:
        #     self.goal = np.float32([0.0, 0.0, -1.0])

        return self.goal

    # Not sure this is nessecary. Could still be able to use scipy's as_euler and as_quat functions
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
    
    def world_err_to_body_err(self, rpy, err_array):
        """Change error in world frame to error in body frame"""
        err_x_body, err_y_body, err_z_body = np.matmul(rpy.as_matrix().transpose(), err_array)
        return err_x_body, err_y_body, err_z_body

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

    def publish_control_data(self, pos, q, pos_d, q_d, body_errors, thrust_body, vel, ang_vel):
        """Publishes control data (pos/att, error, inputs, etc.) to ROS2 topic for analysis"""
        msg = ControlData()
        msg.position = pos
        msg.q = q
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

        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()

        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1

    def traj_timer_callback(self) -> None:
        """Callback function for updating vehicle trajectory"""

        if self.offboard_setpoint_counter >= 11:
            self.now += 0.01
            self.update_goal(self.now)
            self.update_quaternion(self.now)

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