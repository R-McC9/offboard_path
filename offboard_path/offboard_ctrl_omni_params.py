#!/usr/bin/env python

import time
import numpy as np
import math
import pandas as pd
import os

from scipy.spatial.transform import Rotation as R
import rclpy
from rclpy.clock import Clock
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint,
                          VehicleCommand, VehicleControlMode,
                          VehicleLocalPosition, VehicleStatus,
                          VehicleOdometry, VehicleAttitudeSetpoint,
                          VehicleThrustSetpoint, VehicleTorqueSetpoint,
                          ActuatorMotors, ActuatorOutputs)

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
        # self.thrust_setpoint_subscriber = self.create_subscription(
        #     VehicleThrustSetpoint, '/fmu/in/vehicle_thrust_setpoint', self.thrust_setpoint_callback, qos_profile)
        # self.torque_setpoint_subscriber = self.create_subscription(
        #     VehicleTorqueSetpoint, '/fmu/in/vehicle_thrust_setpoint', self.torque_setpoint_callback, qos_profile)
        # self.actuator_motors_subscriber = self.create_subscription(
        #     ActuatorMotors, '/fmu/out/actuator_motors', self.actuator_motors_callback, qos_profile)
        self.actuator_Outputs_subscriber = self.create_subscription(
            ActuatorOutputs, '/fmu/out/actuator_outputs', self.actuator_outputs_callback, qos_profile)

        # Create parameters

        self.declare_parameter('trajectoryCSV', 'UNSET')

        self.trajectory_CSV_name = self.get_parameter('trajectoryCSV').get_parameter_value().string_value
        if self.trajectory_CSV_name == "UNSET":
            raise Exception('No trajectory named '+self.trajectory_CSV_name+' Did you declare your trajectory correclty when launching?')

        wd = os.getcwd()

        self.df = pd.read_csv(wd+"/src/offboard_path/trajectories/"+self.trajectory_CSV_name)

        self.get_logger().info('Set trajectory to: '+self.trajectory_CSV_name)

        # print(df)

        ## Initialize variables ##
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()

        # Create a timer to publish commands
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Create time for faster trajectory updating
        self.now = 0.0
        self.index = 0.0
        self.index_max = len(self.df)
        self.finished = False
        self.traj_timer = self.create_timer(0.01, self.traj_timer_callback)

        self.goal = [0.0, 0.0, -0.0]
        self.q_d = [1.0, 0.0, 0.0, 0.0]

        self.prev_err_x = 0.0
        self.err_sum_x = 0.0

        self.prev_err_y = 0.0
        self.err_sum_y = 0.0

        self.prev_err_z = 0.0
        self.err_sum_z = 0.0

        self.first_run = True
        self.start_pos = [0.0, 0.0, 0.0]

        self.drone_position = [0.0, 0.0, 0.0]
        self.drone_orientation = [1.0, 0.0, 0.0, 0.0]

        self.drone_velocity = [0.0, 0.0, 0.0]
        self.drone_angular_velocity = [0.0, 0.0, 0.0]

        # self.thrust_setpoint = [0.0, 0.0, 0.0]
        # self.torque_setpoint = [0.0, 0.0, 0.0]

        # self.actuator_motors = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.actuator_outputs = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.e_land_triggered = False
        self.saved_pos = False

    def vehicle_odometry_callback(self, msg):
        """Callback function for vehicle_odometry topic subscriber."""
        self.drone_position = msg.position
        self.drone_orientation = msg.q

        self.drone_velocity = msg.velocity
        self.drone_angular_velocity = msg.angular_velocity

        if self.first_run == True:
            self.start_pos = np.float32(np.array(msg.position))
            self.first_run = False

    def PID_controller(self):
        """Callbackfunction for vehicle_odometry topic subscriber."""
        x, y, z = self.drone_position
        w, i, j, k = self.drone_orientation

        kp = 0.8
        ki = 0.006
        kd = 10

        # X, Y position control
        # Convert to body frame from world frame
        # Convert quaternion orientation to euler anglesn (radians)
        rpy = R.from_quat([i, j, k, w])
        # rot_matrix = self.quat2rot(w, i, j, k)
        #roll_meas, pitch_meas, yaw_meas = rpy.as_euler('XYZ', degrees=False)

        err_x = self.goal[0] - x
        err_y = self.goal[1] - y
        err_z = self.goal[2] - z

        # Determine location of the reference position in the body frame
        err_x_body, err_y_body, err_z_body = self.world_err_to_body_err(rpy, np.array([err_x, err_y, err_z]))

        # Separate rotation matrix into lines for publishing, recombine in matlab later
        # rotMat_1 = rotMat[0]
        # rotMat_2 = rotMat[1]
        # rotMat_3 = rotMat[2]

        # Altitude controller
        self.err_sum_z += err_z_body

        err_dif_z = err_z_body - self.prev_err_z

        U_z = kp * err_z_body + ki * self.err_sum_z + kd * err_dif_z

        self.prev_err_z = err_z_body

        # Clamp integral error term when motors are saturated
        Umax = 0.50
        if U_z <= -Umax or U_z >= Umax:
            U_z = np.clip(U_z, Umax, -Umax)
            self.err_sum_z = 0

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
        if U_x <= -Umax or U_x >= Umax:
            U_x = np.clip(U_x, Umax, -Umax)
            self.err_sum_x = 0

        # Clamp integral error term when motors are saturated
        if U_y <= -Umax or U_y >= Umax:
            U_y = np.clip(U_y, Umax, -Umax)
            self.err_sum_y = 0

        # rot_d = R.from_euler('zxy', [0.0, 0.0, 0.0], degrees=True)
        # self.q_d = np.float32(rot_d.as_quat())
        # self.q_d = [0.5, 0.0, 0.0, 0.866025]

        # Publish control inputs
        self.publish_attitude_setpoint(self.q_d, [U_x, U_y, U_z])

        # Publish control data to different topic for graphing/debugging
        self.publish_control_data(self.drone_position, self.drone_orientation, 
                                self.goal, self.q_d, [U_x, U_y, U_z],
                                [err_x_body, err_y_body, err_z_body], self.drone_velocity, self.drone_angular_velocity,
                                self.actuator_outputs)

    def world_err_to_body_err(self, rpy, err_array):
        """Transforms error in world fram to error in body frame"""
        err_x_body, err_y_body, err_z_body = np.matmul(rpy.as_matrix().transpose(), err_array)
        return err_x_body, err_y_body, err_z_body

    def update_quaternion(self, time, index):
        """Updates quaternion setpoint for smooth attitude tracking."""
        # Hold attitude @ 0 pitch/roll/yaw
        # rot_d = R.from_euler('zxy', [0.0, 0.0, 0.0])
        if index < self.index_max:
            csv_quat = self.df.iloc[[index], [4,5,6,7]].to_numpy()

            self.q_d = np.float32(csv_quat[0])
        else:
            self.finished = True

        # print(self.q_d)

        # Rotate about axis tangential to the circle
        # if time < 8.0:
        #     rot_d = R.from_euler('ZXY', [0.0, 0.0, 0.0], degrees=True)
        # elif time > 8.0 and time <= 12.0:
        #     rot_d = R.from_euler('ZXY', [0.0, 0.0, 0.0], degrees=True)
        # elif time > 12.0 and time <= 36.0:
        #     # rotate about a vector tangent to the circle
        #     rot_d = R.from_rotvec(np.pi*2*((time - 12.0)/24.0) * np.array([-1.0*np.cos(2*np.pi*((time - 12.0)/24.0)), 1.0*np.sin(2*np.pi*((time-12.0)/24.0)), 0.0]))
        # elif time > 36.0 and time <= 40.0:
        #     rot_d = R.from_euler('ZXY', [0.0, 0.0, 0.0], degrees=True)
        # elif time > 40.0:
        #     rot_d = R.from_euler('ZXY', [0.0, 0.0, 0.0], degrees=True)

        # Build quaternion from the csv at each time step

        # self.q_d = np.float32([rot_d.as_quat()[3], rot_d.as_quat()[0], rot_d.as_quat()[1], rot_d.as_quat()[2]])
        return self.q_d

# Change trajectories to be a selected CSV on startup. Build python script for trajectory to CSV generation
    def update_goal(self, time, index):
        """Updates position setpoint for smooth position tracking."""
        if index < self.index_max:
            csv_goal = self.df.iloc[[index], [1,2,3]].to_numpy()

            self.goal = np.float32(csv_goal[0])
        else:
            self.finished = True

        # Slowly ascend to 0.8m over 8 seconds and hold
        # if time <= 8.0:
        #     self.goal = np.float32([0.0, 0.0, -0.25 + -0.35*(time/8.0)])
        # else:
        #     self.goal = np.float32([0.0, 0.0, -0.60])

        # Circle @ 1 meter radius
        # if time <= 8.0:
        #     self.goal = [0.0, 0.0, -0.8*(time/8.0)]
        # elif time > 8.0 and time <= 12.0:
        #     self.goal = [-1.0*((time - 8.0)/4.0), 0.0, -0.8]
        # elif time > 12.0 and time <= 36.0:
        #     self.goal = [-1.0*np.cos(2*np.pi*((time - 12.0)/24.0)), 1.0*np.sin(2*np.pi*(time-12.0)/24.0), -0.8]
        # elif time > 36.0 and time <= 40.0:
        #     self.goal = [-1.0 + 1.0*((time - 36.0)/4.0), 0.0, -0.8]
        # else:
        #     self.goal = [0.0, 0.0, -0.8]

        # print(self.df.iloc[[index], [1,2,3]])
        # print(self.goal)

        # build goal from csv at each time step

        return self.goal

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status
    
    # def thrust_setpoint_callback(self, thrust_setpoint):
    #     """Callback function for recording the vehicle thrust setpoint"""
    #     self.thrust_setpoint = thrust_setpoint

    # def torque_setpoint_callback(self, torque_setpoint):
    #     """Callback function for recording the vehicle torque setpoint"""
    #     self.torque_setpoint = torque_setpoint

    # def actuator_motors_callback(self, actuator_motors):
    #     """Callback function for logging motor inputs. Range from [-1, 1]"""
    #     self.actuator_motors = actuator_motors.control

    def actuator_outputs_callback(self, actuators_outputs):
        """Callback function for logging low level controller outputs. Range from [-1, 1]"""
        self.actuator_outputs = actuators_outputs.output

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

    def e_land(self):
        """emergency landing mode for non-vertical orientations"""
        # Save position and orientation at which emergency landing was triggered
        if self.saved_pos == False:
            x0, y0, z0 = self.drone_position
            q0 = self.drone_orientation
            self.saved_pos = True

        t = 0.0
        while self.vehicle_status != 0:
        # Set new goal position and orientation as the current position and orientation,
        # lower drone down over 10 seconds, then disarm
            if t < 2.0:
                self.goal = np.float32(np.array([x0, y0, z0]))
                self.q_d = q0
            elif t >= 2.0:
                self.goal = np.float32(np.array([x0, y0, z0 - z0*((t - 2.0)/10.0)]))
                print(self.drone_position[2])
            self.PID_controller()
            time.sleep(0.01)
            t += 0.01

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

    def publish_control_data(self, pos, q, pos_d, q_d, thrust_body, body_errors, vel, ang_vel, actuator_outputs):
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
        msg.actuator_outputs = actuator_outputs
        # msg.px4_thrust_setpoint = thrust_setpoint
        # msg.px4_torque_setpoint = torque_setpoint
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

        if self.offboard_setpoint_counter >= 11 and self.finished == False:
            self.now += 0.01
            self.PID_controller()
            self.update_goal(self.now, self.index)
            self.update_quaternion(self.now, self.index)
            self.index += 1
        elif self.finished == True:
            raise Exception("End of trajectory csv reached")

def main(args=None) -> None:
    print('Starting offboard control node...')
    rclpy.init(args=args)
    try:
        offboard_control = OffboardControl()
        rclpy.spin(offboard_control)
    except Exception:
        offboard_control.get_logger().info('End of CSV reached, landing')
        offboard_control.land()
    except KeyboardInterrupt:
        offboard_control.get_logger().info('Keyboard interrupt, triggering landing sequence.\n')
        offboard_control.land()
    finally:
        offboard_control.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)