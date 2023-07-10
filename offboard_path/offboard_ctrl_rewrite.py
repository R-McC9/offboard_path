#!/usr/bin/env python

from time import sleep

import rclpy
from rclpy.clock import Clock
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import numpy as np

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
        self.kp = 0.7
        self.ki = 0.01
        self.kd = 30
        
        self.kpx = 1.0
        self.kix = 0.05
        self.kdx = 10
        
        self.kpy = 1.0
        self.kiy = 0.05
        self.kdy = 10
        
        self.kpr = 10.0
        self.kir = 0.0
        self.kdr = 0.0
        
        self.kpp = 10.0
        self.kip = 0.0
        self.kdp = 0.0
        
        self.kpyaw = 0.0
        self.kiyaw = 0.0
        self.kdyaw = 0
        
        
        
        
        

        self.prev_err_x = 0.0
        self.prev_err_y = 0.0
        self.prev_err_z = 0.0
        
        self.prev_err_xb = 0.0
        self.prev_err_yb = 0.0
        self.prev_err_zb = 0.0

        self.err_sum_x = 0.0
        self.err_sum_y = 0.0
        self.err_sum_z = 0.0
        
        self.err_sum_x_body = 0.0
        self.err_sum_y_body = 0.0
        
        self.err_sum_roll = 0.0
        self.prev_err_roll = 0.0
        
        self.err_sum_pitch = 0.0
        self.prev_err_pitch = 0.0
        
        self.err_sum_yaw = 0.0
        self.prev_err_yaw = 0.0
        
        self.goal = [0.0, 0.0, -4.0]

    def odometry_callback(self, msg):
        #recieves odometry data, does PID control math, publishes correct thrust and angular rates
        #print('This is a test, are we running this callback function?') 
        #self.get_logger().info('Recieveing messages from odometry topic, %s' % msg.position)

        x, y, z = msg.position
        
        #roll_meas, pitch_meas, yaw_meas = msg.q
        w, i, j, k = msg.q 
        
        
        
        import math
 
        def euler_from_quaternion(x, y, z, w):
       
            t0 = +2.0 * (w * x + y * z)
            t1 = +1.0 - 2.0 * (x * x + y * y)
            roll_x = math.atan2(t0, t1)
        
            t2 = +2.0 * (w * y - z * x)
            t2 = +1.0 if t2 > +1.0 else t2
            t2 = -1.0 if t2 < -1.0 else t2
            pitch_y = math.asin(t2)
        
            t3 = +2.0 * (w * z + x * y)
            t4 = +1.0 - 2.0 * (y * y + z * z)
            yaw_z = math.atan2(t3, t4)
        
            return roll_x, pitch_y, yaw_z # in radians
            
        roll_meas, pitch_meas, yaw_meas = euler_from_quaternion(i, j, k, w)
        
        x_ref = self.goal[0]
        y_ref = self.goal[1]
      	
        err_x = self.goal[0] - x
        err_y = self.goal[1] - y
        err_z = self.goal[2] - z
      	
        def world_to_body(yaw, err_x, err_y):
            x_bref = math.cos(yaw)*err_x - math.sin(yaw)*err_y
            y_bref = math.sin(yaw)*err_x + math.cos(yaw)*err_y
            print(x_bref, y_bref)
            return y_bref, x_bref
      
     	#roll and pitch controller goes from body to roll
     	
        y_bref, x_bref = world_to_body(yaw_meas, err_x, err_y)
        
      	
        self.err_sum_x_body += x_bref	
        self.err_sum_y_body += y_bref	
      	
        err_dif_xb = x_bref - self.prev_err_xb
        err_dif_yb = y_bref - self.prev_err_xb
      	

        roll = self.kpy * y_bref + self.kiy * self.err_sum_y_body + self.kdy*err_dif_yb
        pitch = self.kpx * x_bref + self.kpx * self.err_sum_x_body + self.kdx*err_dif_xb
        
        self.prev_err_xb = x_bref
        self.prev_err_yb = y_bref

        #print("error between cutternt z and goal z = %s" % err_z)
        
        #roll controller 2
        
        roll_ref = roll
        
        
        roll_err = roll_ref - roll_meas 
        
        self.err_sum_roll += roll_err
        
        err_dif_roll = roll_err-self.prev_err_roll
        
        roll_rate = self.kpr * roll_err + self.kir * self.err_sum_roll + self.kdr * err_dif_roll
        
        self.prev_err_roll = roll_err
        
        #pitch controller
        
        pitch_ref = pitch
        
        
        pitch_err = pitch_ref - pitch_meas 
        
        self.err_sum_pitch += pitch_err
        
        err_dif_pitch = pitch_err-self.prev_err_pitch
        
        pitch_rate = self.kpp * pitch_err + self.kip * self.err_sum_pitch + self.kdp * err_dif_pitch
        
        self.prev_err_roll = roll_err
        
        #yaw controller
        
        yaw_ref = 0
        
        
        yaw_err = yaw_ref - yaw_meas 
        
        self.err_sum_yaw += yaw_err
        
        err_dif_yaw = yaw_err-self.prev_err_yaw
        
        yaw_rate = self.kpy * yaw_err + self.kiy * self.err_sum_yaw + self.kdy * err_dif_yaw
        
        
        
        self.prev_err_yaw = yaw_err
        
        
        #Altitude Controller

        self.err_sum_x += err_x
        self.err_sum_y += err_y
        self.err_sum_z += err_z

        err_dif_x = err_x - self.prev_err_x
        err_dif_y = err_y - self.prev_err_y
        err_dif_z = err_z - self.prev_err_z

        U_x = self.kp * err_x + self.ki * self.err_sum_x + self.kd * err_dif_x
        U_y = self.kp * err_y + self.ki * self.err_sum_y + self.kd * err_dif_y
        U_z = self.kp * err_z + self.ki * self.err_sum_z + self.kd * err_dif_z

        if U_z <= -1.0:
            # When saturated keep integral term constant
            U_z = -1.0
            self.err_sum_z = 0

        if U_z >= 1.0:
            U_z = 1.0
            self.err_sum_z = 0

        thrust_rates = [0.0, 0.0, U_z]

        self.prev_err_x = err_x
        self.prev_err_y = err_y
        self.prev_err_z = err_z

        #Compute thrust input using PID controller
        #print(self.U_z)
        #print('got here')
        
        ang_rates = [roll_rate, pitch_rate, yaw_rate]
        #print(roll, pitch)
        
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
        # msg.roll = 0.0
        # msg.yaw = 0.0
        # msg.pitch = 0.0        
        msg.roll = ang_rates[0]
        msg.pitch = ang_rates[1]
        msg.yaw = 0.0
        msg.thrust_body = [0.0, 0.0, thrust_rates[2]]
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