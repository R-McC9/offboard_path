import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleOdometry, VehicleLocalPosition, VehicleStatus

from offboard_msgs.msg import ControlData

import numpy as np

class OffboardControl(Node):
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self) -> None:
        super().__init__('offboard_control_takeoff_and_land')

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        self.control_data_publisher = self.create_publisher(
            ControlData, '/ControlData', qos_profile)

        # Create subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        self.vehicle_odometry_subscriber = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', self.vehicle_odometry_callback, qos_profile)

        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_odometry = VehicleOdometry()
        self.vehicle_status = VehicleStatus()
        self.takeoff_height = -0.8

        # Create a timer to publish control commands
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Create time for smooth trajectory tracking
        self.traj_timer = self.create_timer(0.01, self.traj_timer_callback)

        self.goal = [0.0, 0.0, 0.0]
        self.start_pos = [0.0, 0.0, 0.0]
        self.now = 0.0

    def update_goal(self, time):
        """Updates goal position for smooth trajectory tracking"""
        # Slowly ascend to 0.8m over 8 seconds and hold
        # if time <= 8.0:
        #     self.goal = [0.0, 0.0, -0.8*(time/8.0)]
        # else:
        #     self.goal = [0.0, 0.0, -0.8]

        # Ascend, slide right then left
        # if time <= 8.0:
        #     self.goal = [0.0, 0.0, -0.8]
        # elif time > 8.0 and time <= 12.0:
        #     self.goal = [0.0, 1.0*((time - 8.0)/4.0), -0.8]
        # elif time > 12.0 and time <= 20.0:
        #     self.goal = [0.0, 1.0 - 2.0*((time - 12.0)/8.0), -0.8]
        # elif time > 20.0 and time <= 24.0:
        #     self.goal = [0.0, -1.0 + 1.0*((time - 20.0)/4.0), -0.8]
        # else:
        #     self.goal = [0.0, 0.0, -0.8]

        #Ascend, slide forward then backward
        # if time <= 8.0:
        #     self.goal = [0.0, 0.0, -0.8]
        # elif time > 8.0 and time <= 12.0:
        #     self.goal = [1.0*((time - 8.0)/4.0), 0.0, -0.8]
        # elif time > 12.0 and time <= 20.0:
        #     self.goal = [1.0 - 2.0*((time - 12.0)/8.0), 0.0, -0.8]
        # elif time > 20.0 and time <= 24.0:
        #     self.goal = [-1.0 + 1.0*((time - 20.0)/4.0), 0.0, -0.8]
        # else:
        #     self.goal = [0.0, 0.0, -0.8]

        # Slowly ascend, trace a circle with 1 meter radius
        if time <= 8.0:
            self.goal = [0.0, 0.0, -0.8*(time/8.0)]
        elif time > 8.0 and time <= 12.0:
            self.goal = [-1.0*((time - 8.0)/4.0), 0.0, -0.8]
        elif time > 12.0 and time <= 36.0:
            self.goal = [-1.0*np.cos(2*np.pi*((time - 12.0)/24.0)), 1.0*np.sin(2*np.pi*(time-12.0)/24.0), -0.8]
        elif time > 36.0 and time <= 40.0:
            self.goal = [-1.0 + 1.0*((time - 36.0)/4.0), 0.0, -0.8]
        else:
            self.goal = [0.0, 0.0, -0.8]

        # Vertical Figure 8
        # if time <= 8.0:
        #     self.goal = [0.0, 0.0, -1.0*(time/8.0)]
        # elif time > 8.0 and time <= 30.0:
        #     self.goal = [(np.cos(2*np.pi*((time - 8.0)/22.0) + (np.pi/2))) / (1.0 + np.sin(2*np.pi*((time - 8.0)/22.0) + (np.pi/2))**2), 
        #                             0.0, 
        #                             -1.0 + (np.sin(2*np.pi*((time - 8.0)/22.0) + (np.pi/2)) * np.cos(2*np.pi*((time - 8.0)/22.0) + (np.pi/2))) / (1.0 + np.sin(2*np.pi*((time - 8.0)/22.0) + (np.pi/2))**2)]
        # else:
        #     self.goal = [0.0, 0.0, -1.0]

    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position

    def vehicle_odometry_callback(self, vehicle_odometry):
        """Callback function for vehicle_odometry topic subscriber."""
        self.vehicle_odometry = vehicle_odometry

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
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self, x: float, y: float, z: float):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 0.0
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        #self.get_logger().info(f"Publishing position setpoints {[x, y, z]}")

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

    def publish_control_data(self, vehicle_odometry, goal):
        """Publishes control data (pos/att, error, inputs, etc.) to ROS2 topic for analysis"""
        msg = ControlData()
        msg.position = vehicle_odometry.position
        msg.q = vehicle_odometry.q
        msg.position_d = goal
        # msg.q_d = q_d
        # msg.body_errors = body_errors
        msg.velocity = vehicle_odometry.velocity
        msg.angular_velocity = vehicle_odometry.angular_velocity
        #msg.body_thrust_inputs = thrust_body
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.control_data_publisher.publish(msg)

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

    def traj_timer_callback(self) -> None:
        """Callback for updating trajecotry setpoint"""

        if self.offboard_setpoint_counter >= 11:
            self.now += 0.01
            self.update_goal(self.now)
            self.publish_position_setpoint(self.goal[0], self.goal[1], self.goal[2])
            self.publish_control_data(self.vehicle_odometry, self.goal)

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
