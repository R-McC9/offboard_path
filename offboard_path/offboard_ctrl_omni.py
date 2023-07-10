#!/usr/bin/env python

from time import sleep

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

        #Need custom QoS profile!!!!!
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.offboard_control_mode_publisher_ = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", qos_profile)
        self.trajectory_setpoint_publisher_ = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos_profile)
        self.rates_setpoint_publisher_ = self.create_publisher(VehicleRatesSetpoint, "/fmu/in/vehicle_rates_setpoint", qos_profile)
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", qos_profile)

        # self.timesync_sub_ = self.create_subscription(TimesyncStatus, "/fmu/out/timesync_status", 10)

        self.vehicle_odometry_subscriber_ = self.create_subscription(VehicleOdometry, "/fmu/out/vehicle_odometry", self.odometry_callback, qos_profile)

        self.offboard_setpoint_counter = 0

        timer_period = 0.1
        self.timer_ = self.create_timer(timer_period, self.timer_callback)

        #PID controller parameters
        self.kp = 0.3
        self.ki = 0.000001
        self.kd = 5.0
        #INstantiate PID controller variables
        self.prev_err_x = 0.0
        self.prev_err_y = 0.0
        self.prev_err_z = 0.0

        self.err_sum_x = 0.0
        self.err_sum_y = 0.0
        self.err_sum_z = 0.0

        self.goal = [0.0, 0.0, -6.0]

    def odometry_callback(self, msg):
        #recieves odometry data, does PID control math, publishes correct thrust and angular rates
        #self.get_logger().info('Recieveing messages from odometry topic, %s' % msg.position)

        #Get current position
        x, y, z = msg.position

        #Get current orientation (quaternion)
        q, i, j, k = msg.q

        #Compute current error
        err_x = self.goal[0] - x
        err_y = self.goal[1] - y
        err_z = self.goal[2] - z
        #Compute error sum for integral term
        self.err_sum_x += err_x
        self.err_sum_y += err_y
        self.err_sum_z += err_z
        #Compute error difference for derivative term
        err_dif_x = err_x - self.prev_err_x
        err_dif_y = err_y - self.prev_err_y
        err_dif_z = err_z - self.prev_err_z
        #Compute control inputs
        U_x = self.kp * err_x + self.ki * self.err_sum_x + self.kd * err_dif_x
        U_y = self.kp * err_y + self.ki * self.err_sum_y + self.kd * err_dif_y
        U_z = self.kp * err_z + self.ki * self.err_sum_z + self.kd * err_dif_z

        if U_x <= -1.0:
            #When saturated lock integral term 
            U_x = -1.0
            self.err_sum_x = 0

        if U_x >= 1.0:
            U_x = 1.0
            self.err_sum_x = 0

        if U_y <= -1.0:
            #When saturated lock integral term 
            U_y = -1.0
            self.err_sum_y = 0

        if U_y >= 1.0:
            U_y = 1.0
            self.err_sum_y = 0

        if U_z <= -1.0:
            #When saturated lock integral term 
            U_z = -1.0
            self.err_sum_z = 0

        if U_z >= 1.0:
            U_z = 1.0
            self.err_sum_z = 0

        thrust_rates = [U_x, U_y, U_z]
        print(U_x)

        self.prev_err_x = err_x
        self.prev_err_y = err_y
        self.prev_err_z = err_z

        ang_rates = [0.0, 0.0, 0.0]

        #Compute thrust input using PID controller math
        #print(self.U_z)
        self.publish_rates_setpoint(ang_rates, thrust_rates)
        

    def timer_callback(self):
        if (self.offboard_setpoint_counter == 10):
            # Change to offboard mode after 10 setpoints
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)
            # Arm the vehicle
            self.arm()

        self.publish_offboard_control_mode()
        #self.publish_trajectory_setpoint([0.0, 0.0, -3.0])
        #self.publish_rates_setpoint(None, -1.0)

        # Stop counter after reaching 11
        if (self.offboard_setpoint_counter < 11):
            self.offboard_setpoint_counter += 1

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info("Arm command send")

    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self.get_logger().info("Disarm command send")

    def land(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND, 21.)
        sleep(3)

    def publish_offboard_control_mode(self):
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
        msg = VehicleRatesSetpoint()
        msg.roll = ang_rates[0]
        msg.yaw = ang_rates[1]
        msg.pitch = ang_rates[2]
        #msg.thrust_body = [0.0, 0.0, thrust_rates[2]]
        msg.thrust_body = thrust_rates
        self.rates_setpoint_publisher_.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher_.publish(msg)        

def main(args=None):
    rclpy.init(args=args)
    offboard_control = OffboardControl()

    try:
        offboard_control.get_logger().info('Starting offboard control node, shut down with CTRL-C')
        rclpy.spin(offboard_control)
    except KeyboardInterrupt:
        offboard_control.get_logger().info('Keyboard interrupt, shutting down.\n')
    #Execute Landing
    offboard_control.land()
    #Destroy node explicitly on keyboard interrupt
    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
