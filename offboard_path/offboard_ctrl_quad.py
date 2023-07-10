#!/usr/bin/env python

from time import sleep
import math
import numpy as np

import rclpy
from rclpy.clock import Clock
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint,
                          VehicleCommand, VehicleControlMode, VehicleOdometry,
                          VehicleRatesSetpoint)

class OffboardControl(Node):

    def __init__(self):
        # Create node and publishers/subscribers
        super().__init__('Offboard_Ctrl')

        # Define custom QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create Publishers
        self.offboard_control_mode_publisher_ = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", qos_profile)
        self.trajectory_setpoint_publisher_ = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos_profile)
        self.rates_setpoint_publisher_ = self.create_publisher(
            VehicleRatesSetpoint, "/fmu/in/vehicle_rates_setpoint", qos_profile)
        self.vehicle_command_publisher_ = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", qos_profile)

        # Create Subscribers
        self.vehicle_odometry_subscriber_ = self.create_subscription(
            VehicleOdometry, "/fmu/out/vehicle_odometry", self.odometry_callback, qos_profile)

        self.offboard_setpoint_counter = 0

        timer_period = 0.1
        self.timer_ = self.create_timer(timer_period, self.timer_callback)

        # PID controller parameters (tune bfore testing on hardware)
        # Altitude Controller
        self.kp = 0.7
        self.ki = 0.001
        self.kd = 30

        # Position controller
        # roll/pitch setpoint (body frame)
        self.kpx = 1.0
        self.kix = 0.000000001
        self.kdx = 10.000000001

        self.kpy = 1.0
        self.kiy = 0.000000001
        self.kdy = 10.0

        # rates setpoints
        self.kpr = 0.0000005
        self.kir = 0.03
        self.kdr = 100.0

        self.kpp = 0.0000005
        self.kip = 0.03
        self.kdp = 100.0

        # Instantiate error parameters
        # World frame
        self.prev_err_x = 0.0
        self.prev_err_y = 0.0
        self.prev_err_z = 0.0

        self.err_sum_x = 0.0
        self.err_sum_y = 0.0
        self.err_sum_z = 0.0

        # Body Frame
        self.prev_err_x_body = 0.0
        self.prev_err_y_body = 0.0

        self.err_sum_x_body = 0.0
        self.err_sum_y_body = 0.0

        self.err_sum_roll = 0.0
        self.prev_err_roll = 0.0
        
        self.err_sum_pitch = 0.0
        self.prev_err_pitch = 0.0
        
        self.err_sum_yaw = 0.0
        self.prev_err_yaw = 0.0

        # Define goal position, this needs to be a trajectory in the future!!!
        self.goal = [4.0, 3.0, -10.0]

    def odometry_callback(self, msg):
        """Recieves odometry data, does PID control math, publishes thrust and angular rates as inputs"""
        # Get current position
        x, y, z = msg.position

        # Get current orientation (quaternion)
        # Drone is in NED frame (does this matter??)
        w, i, j, k = msg.q

        # Nested PID loops for x,y position tracking using pitch/roll/yaw rates
        # If we could thrust in all directions (omnidirectional) then this could work by default just using U_x and U_y, would just need to control attitude then.
        # Determine x,y position errors
        err_x = self.goal[0] - x
        err_y = self.goal[1] - y

        self.err_sum_x += err_x
        self.err_sum_y += err_y

        err_dif_x = err_x - self.prev_err_x
        err_dif_y = err_y - self.prev_err_y

        U_x = self.kp * err_x + self.ki * self.err_sum_x + self.kd * err_dif_x
        U_y = self.kp * err_y + self.ki * self.err_sum_y + self.kd * err_dif_y

        self.prev_err_x = err_x
        self.prev_err_y = err_y

        # Need pitch and roll setpoints for translation
        # Convert quaternion orientation to euler anglesn (radians)
        roll_meas, pitch_meas, yaw_meas = self.euler_from_quaternion(w, i, j, k)

        # Determine position of reference in the body frame
        err_x_body, err_y_body = self.world_err_to_body_err(yaw_meas, err_x, err_y)

        # Determine angle to pitch and/or roll to to move towards reference trajectory (body frame)
        self.err_sum_x_body += err_x_body
        self.err_sum_y_body += err_y_body

        err_dif_x_body = err_x_body - self.prev_err_x_body
        err_dif_y_body = err_y_body - self.prev_err_y_body

        roll = self.kpx * err_x_body + self.kix * self.err_sum_x_body + self.kdx * err_dif_x_body
        pitch = -1*(self.kpy * err_y_body + self.kiy * self.err_sum_y_body + self.kdy * err_dif_y_body)

        # Remove this lateer!!!
        # Bypassing controller for now
        # +roll is right, -roll is left
        # +pitch is back, -pitch is forward
        # roll = 0
        # pitch = -5

        # Clamping terms to avoid over pitching/rolling
        if pitch <= -5:
            pitch = -5
            self.err_sum_pitch = 0

        if pitch >= 5:
            pitch = 5
            self.err_sum_pitch = 0

        if roll <= -5:
            roll = -5
            self.err_sum_roll = 0

        if roll >= 5:
            roll = 5
            self.err_sum_roll = 0

        self.prev_err_x_body = err_x_body
        self.prev_err_y_body = err_y_body

        # PID controllers to track roll and pitch setpoints

        # +roll_rate = right, -roll_rate = left
        # +pitch_rate = backward, -pitch_rate = forward
        # Roll
        roll_ref = roll

        roll_err = roll_ref - roll_meas

        self.err_sum_roll += roll_err

        err_dif_roll = roll_err  - self.prev_err_roll

        roll_rate = -1*(self.kpr * roll_err + self.kir * self.err_sum_roll + self.kdr * err_dif_roll)

        if roll_rate <= -0.1:
            roll_rate = -0.1
            self.err_sum_roll = 0

        if roll_rate >= 0.1:
            roll_rate = 0.1
            self.err_sum_roll = 0

        self.prev_err_roll = roll_err
        
        # Pitch
        pitch_ref = pitch

        pitch_err = pitch_ref - pitch_meas

        self.err_sum_pitch += pitch_err

        err_dif_pitch = pitch_err - self.prev_err_pitch

        pitch_rate = self.kpp * pitch_err + self.kip * self.err_sum_pitch + self.kdp * err_dif_pitch

        if pitch_rate <= -0.1:
            pitch_rate = -0.1
            self.err_sum_pitch = 0

        if pitch_rate >= 0.1:
            pitch_rate = 0.1
            self.err_sum_pitch = 0

        self.prev_err_pitch = pitch_err

        #print(self.goal[0], err_x_body, roll_rate)
        #print(self.goal[1], err_y_body, pitch_rate)
        #print(x, y, roll, pitch, yaw_meas)
        print(x, y)

        # Yaw
        yaw_ref = 0.0

        yaw_err = yaw_ref - yaw_meas

        self.err_sum_yaw += yaw_err

        err_dif_yaw = yaw_err - self.prev_err_yaw

        yaw_rate = self.kpy * yaw_err + self.kiy * self.err_sum_yaw + self.kdy * err_dif_yaw

        self.prev_err_yaw = yaw_err
        # yaw_rate = 0.0

        # Altitude controller
        err_z = self.goal[2] - z

        self.err_sum_z += err_z

        err_dif_z = err_z - self.prev_err_z

        U_z = self.kp * err_z + self.ki * self.err_sum_z + self.kd * err_dif_z

        # Clamp integral error term when motors are saturated
        if U_z <= -1.0:
            U_z = -1.0
            self.err_sum_z = 0

        if U_z >= 1.0:
            U_z = 1.0
            self.err_sum_z = 0

        self.prev_err_z = err_z

        thrust_rates = [0.0, 0.0, U_z]
        ang_rates = [roll_rate, pitch_rate, 0.0]

        # Compute thrust input using PID controller math
        self.publish_rates_setpoint(ang_rates, thrust_rates)

    def euler_from_quaternion(self, w, i, j, k):
        jsqr = j * j
       
        t0 = +2.0 * (w * i + j * k)
        t1 = +1.0 - 2.0 * (i * i + jsqr)
        roll_x = np.radians(np.arctan2(t0, t1))
        
        t2 = +2.0 * (w * j - k * i)
        # t2 = +1.0 if t2 > +1.0 else t2
        # t2 = -1.0 if t2 < -1.0 else t2
        t2 = np.where(t2>+1.0, +1.0, t2)
        t2 = np.where(t2<-1.0, -1.0, t2)
        pitch_y = np.radians(np.arcsin(t2))
        
        t3 = +2.0 * (w * k + i * j)
        t4 = +1.0 - 2.0 * (jsqr + k * k)
        yaw_z = np.radians(np.arctan2(t3, t4))
        
        return roll_x, pitch_y, yaw_z # in radians

    def world_err_to_body_err(self, yaw_meas, err_x, err_y):
        x_err_body = math.cos(yaw_meas)*err_x - math.sin(yaw_meas)*err_y
        y_err_body = math.sin(yaw_meas)*err_x + math.cos(yaw_meas)*err_y
        return x_err_body, y_err_body

    def timer_callback(self):
        """Timer callback function"""
        if (self.offboard_setpoint_counter == 10):
            # Change to offboard mode after 10 setpoints
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
            # Arm the vehicle
            self.arm()

        self.publish_offboard_control_mode()
        #self.publish_trajectory_setpoint([0.0, 0.0, -3.0])
        #self.publish_rates_setpoint(None, -1.0)

        # Stop counter after reaching 11
        if (self.offboard_setpoint_counter < 11):
            self.offboard_setpoint_counter += 1


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

    def land(self):
        """Send a land commadn to the vehicle"""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_NAV_LAND, param1=21.0)
        self.get_logger().info("Landing...")
        sleep(3)

    def publish_offboard_control_mode(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = True
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher_.publish(msg)

    def publish_trajectory_setpoint(self, desiredXYZ):
        msg = TrajectorySetpoint()
        msg.position = desiredXYZ
        msg.yaw = 0.0
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher_.publish(msg)

    def publish_rates_setpoint(self, ang_rates, thrust_rates):
        """Publish rates setpoint"""
        msg = VehicleRatesSetpoint()
        # msg.roll = 0.0
        # msg.yaw = 0.0
        # msg.pitch = 0.0
        msg.roll = ang_rates[0]
        msg.pitch = ang_rates[1]
        msg.yaw = ang_rates[2]
        # msg.thrust_body = [0.0, 0.0, thrust_rates[2]]
        msg.thrust_body = thrust_rates
        self.rates_setpoint_publisher_.publish(msg)

    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command (arm, disarm, etc.)"""
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
        self.vehicle_command_publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    offboard_control = OffboardControl()

    try:
        offboard_control.get_logger().info('Starting offboard control node, shut down with CTRL-C')
        rclpy.spin(offboard_control)
    except KeyboardInterrupt:
        offboard_control.get_logger().info('Keyboard interrupt, shutting down.\n')
    # Execute Landing
    offboard_control.land()
    # Destroy node explicitly on keyboard interrupt
    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
