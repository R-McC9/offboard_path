#!/usr/bin/env python

import time
import numpy as np
import math
from scipy.spatial.transform import Rotation as R
import rclpy
from rclpy.clock import Clock
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import Point, Quaternion

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

        # Create client for updating fake_sat in Gazebo
        # self.cli = self.create_client(SetEntityState, 'set_entity_state')

        # Create Subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_odometry_subscriber = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', self.vehicle_odometry_callback, qos_profile)
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        # self.actuator_motors_subscriber = self.create_subscription(
        #     ActuatorMotors, '/fmu/out/actuator_motors', self.actuator_motors_callback, qos_profile)
        self.actuator_Outputs_subscriber = self.create_subscription(
            ActuatorOutputs, '/fmu/out/actuator_outputs', self.actuator_outputs_callback, qos_profile)

        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()

        # Create a timer to publish commands
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Create time for faster trajectory updating
        self.now = 0.0
        self.traj_timer = self.create_timer(0.01, self.traj_timer_callback)

        self.goal = [0.0, 0.0, -0.0]
        self.q_d = [1.0, 0.0, 0.0, 0.0]

        # Ficticious satellite parameters
        self.delta_x = 0.0
        self.delta_y = 0.0
        self.delta_z = 0.0

        self.ddotX = 0.0
        self.ddotY = 0.0
        self.ddotZ = 0.0

        self.xv = 0.0
        self.yv = 0.0
        self.zv = 0.0

        self.prev_err = [0.0, 0.0, 0.0]
        self.err_sum = [0.0, 0.0, 0.0]

        # Drone PId controller params
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

        self.drone_acceleration = [0.0, 0.0, 0.0]

        # self.thrust_setpoint = [0.0, 0.0, 0.0]
        # self.torque_setpoint = [0.0, 0.0, 0.0]

        # self.actuator_motors = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.actuator_outputs = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.e_land_triggered = False
        self.saved_pos = False

    def vehicle_odometry_callback(self, msg):
        """Callback function for vehicle_odometry topic subscriber."""
        # self.drone_position = msg.position
        # self.droce_acceleration = msg.acceleration
        self.drone_orientation = msg.q

        # self.drone_velocity = msg.velocity
        self.drone_angular_velocity = msg.angular_velocity

        if self.first_run == True:
            self.start_pos = np.float32(np.array(msg.position))
            self.first_run = False

    def vehicle_local_position_callback(self, msg):
        """Callback function for vehicle_local_position topic subscriber"""
        self.drone_position = [msg.x, msg.y, msg.z]
        self.drone_velocity = [msg.vx, msg.vy, msg.vz]
        self.drone_acceleration = [msg.ax, msg.ay, msg.az]

        # print(self.drone_position)

    def PID_controller(self):
        """Callbackfunction for vehicle_odometry topic subscriber."""
        # drone_position is position relative to center of world
        x, y, z = self.drone_position
        w, i, j, k = self.drone_orientation

        # PID constants for position control
        kp = 0.8
        ki = 0.006
        kd = 10

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

        # # Altitude controller
        # self.err_sum_z += err_z_body

        # err_dif_z = err_z_body - self.prev_err_z

        # U_z = kp * err_z_body + ki * self.err_sum_z + kd * err_dif_z

        # self.prev_err_z = err_z_body

        # X,Y position controller
        self.err_sum_x += err_x_body
        self.err_sum_y += err_y_body
        self.err_sum_z += err_z_body

        err_dif_x = err_x_body - self.prev_err_x
        err_dif_y = err_y_body - self.prev_err_y
        err_dif_z = err_z_body - self.prev_err_z

        U_x = kp * err_x_body + ki * self.err_sum_x + kd * err_dif_x
        U_y = kp * err_y_body + ki * self.err_sum_y + kd * err_dif_y
        U_z = kp * err_z_body + ki * self.err_sum_z + kd * err_dif_z

        self.prev_err_x = err_x_body
        self.prev_err_y = err_y_body
        self.prev_err_z = err_z_body

        # Clamp integral error term when motors are saturated
        Umax = 0.50
        if U_x <= -Umax or U_x >= Umax:
            U_x = np.clip(U_x, Umax, -Umax)
            self.err_sum_x = 0

        # Clamp integral error term when motors are saturated
        if U_y <= -Umax or U_y >= Umax:
            U_y = np.clip(U_y, Umax, -Umax)
            self.err_sum_y = 0

        # Clamp integral error term when motors are saturated
        if U_z <= -Umax or U_z >= Umax:
            U_z = np.clip(U_z, Umax, -Umax)
            self.err_sum_z = 0

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

    def update_quaternion(self, time):
        """Updates quaternion setpoint for smooth attitude tracking."""
        # Hold attitude @ 0 pitch/roll/yaw
        rot_d = R.from_euler('zxy', [0.0, 0.0, 0.0])

        self.q_d = np.float32([rot_d.as_quat()[3], rot_d.as_quat()[0], rot_d.as_quat()[1], rot_d.as_quat()[2]])
        return self.q_d

# Change trajectories to be a selected CSV on startup. Build python script for trajectory to CSV generation
    def update_goal(self, time):
        """Updates position setpoint for smooth position tracking."""
        #position of chief satellite: 100 meters away from the origin in the x direction, 1 meter upwards
        # THE CHIEF IS NOT ROTATED RELATIVE TO THE WORLD FRAME
        x_c, y_c, z_c = [0.0, 0.0, -5.0]
        # position of drone in world frame
        # x, y, z = self.drone_position

        # Ficticious drone parameters:
        # We are pretending the drone is a satellite of some mass, and produces forces to change its position.
        # mass, kg
        m = 60

        # Compute n for the deputy satellite
        mu = 3.986E14
        r_0 = 200000
        n = np.sqrt(mu/(r_0 + x_c + self.goal[0])**3)

        # Slowly ascend to 1m over 8 seconds, change goal position based on deltas.
        if time <= 10.0:
            self.goal = np.float32([-1.0*(time/10.0), 0.0, -0.25 + -4.75*(time/10.0)])
        elif time > 10.0:

            # # Fictitious satellite PID position controller:
            # kp = 0.01
            # ki = 0.0005
            # kd = 50.0

            # # desired acceleration relative to the chief
            # acc_des = [0.0, 0.0, 0.0]
            # err = [acc_des[0] - self.ddotX, acc_des[1] - self.ddotY, acc_des[2] - self.ddotY]

            # self.err_sum += err

            # Ux = kp*err[0] + ki*self.err_sum[0] + kd*(err[0] - self.prev_err[0])
            # Uy = kp*err[1] + ki*self.err_sum[1] + kd*(err[1] - self.prev_err[1])
            # Uz = kp*err[2] + ki*self.err_sum[2] + kd*(err[2] - self.prev_err[2])

            # self.prev_err = err

            Ux = 0.0
            Uy = 0.0
            Uz = 0.0
            
            # Find the acceleration relative to the chief due to the differences in orbit
            # need position of deputy relative to chief (x_c - self.goal[0])
            self.ddotX = 3*n**2 * (self.goal[0]) + 2*n * self.yv + 1/m * Ux
            self.ddotY = -2*n*self.xv + 1/m * Uy
            self.ddotZ = -1*n**2 * (z_c-self.goal[2]) + 1/m * Uz

            # Calculate where the drone should be located,
            # knowing what we know about the deputy's position relative to the chief, using kinematics:
            self.delta_x += self.xv*0.01 + 0.5*self.ddotX*(0.01**2)
            self.delta_y += self.yv*0.01 + 0.5*self.ddotY*(0.01**2)
            self.delta_z += self.zv*0.01 + 0.5*self.ddotZ*(0.01**2)

            # update velocity term for next itteration
            self.xv += self.ddotX*0.01
            self.yv += self.ddotY*0.01
            self.zv += self.ddotZ*0.01

            self.goal = np.float32([-1.0+self.delta_x, 0.0+self.delta_y, -5.0+self.delta_z])

        # print(self.goal)

        # self.publish_fake_sat_position(self.goal[0], self.goal[1], self.goal[2])

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

    # def publish_fake_sat_position(self, x, y, z):
    #     self.req = SetEntityState.Request()
    #     pos = Point()
    #     pos.x = np.float(x)
    #     pos.y = np.float(y)
    #     pos.z = np.float(z)

    #     quat = Quaternion()
    #     quat.w = 1.0
    #     quat.x = 0.0
    #     quat.y = 0.0
    #     quat.z = 0.0

    #     self.req.state.name = 'fake_sat'
    #     self.req.state.pose.position = pos
    #     self.req.state.pose.orientation = quat
    #     self.future = self.cli.call_async(self.req)
    #     rclpy.spin_until_future_complete(self, self.future)

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
            self.PID_controller()
            self.update_goal(self.now)
            self.update_quaternion(self.now)

def main(args=None) -> None:
    print('Starting offboard control node...')
    rclpy.init(args=args)
    try:
        offboard_control = OffboardControl()
        rclpy.spin(offboard_control)
    except KeyboardInterrupt:
        offboard_control.get_logger().info('Keyboard interrupt, triggering landing sequence.\n')

    offboard_control.land()
    #offboard_control.e_land()
    offboard_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)