#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
import serial
import threading
import math
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster

class AMRNode(Node):
    def __init__(self):
        super().__init__('gati_bot_node')

        # --- SERIAL CONFIGURATION ---
        serial_port = '/dev/gati_bot_mb' 
        try:
            self.ser = serial.Serial(serial_port, 115200, timeout=0.05)
            self.ser_lock = threading.Lock() # Prevents collision between read/write
            self.get_logger().info(f"Connected to {serial_port}")
        except Exception as e:
            self.get_logger().error(f"Serial Error: {e}")
            exit(1)

        # --- ROBOT PARAMETERS ---
        self.wheel_radius = 0.0325    # 32.5mm (65mm diameter)
        self.wheel_base   = 0.134  #0.134     # Distance between wheels (meters)
        self.ticks_per_rev = 275.0    # Encoder ticks per wheel revolution
        
        # Max physical speed of your robot (m/s) 
        # Increase this if the robot moves too slow, decrease if it's too fast
        self.max_speed = 0.5 

        # --- ODOMETRY STATE ---
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.prev_l = None
        self.prev_r = None
        self.last_time = self.get_clock().now()

        # -- ROS2 SUBSCRIBER FOR CMD_VEL ---
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # -- Odom Publisher and TF Broadcaster ---
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Start serial reading thread
        threading.Thread(target=self.read_serial_loop, daemon=True).start()

    def cmd_vel_callback(self, msg: Twist):
        """ Converts Twist messages to Differential Drive wheel speeds """
        v = msg.linear.x
        w = msg.angular.z

        # Differential Drive Kinematics
        left_vel  = v - (w * self.wheel_base / 2.0)
        right_vel = v + (w * self.wheel_base / 2.0)

        # Map velocity to -1.0 to 1.0 range based on max_speed
        left_pwm  = left_vel / self.max_speed
        right_pwm = right_vel / self.max_speed

        # Constrain to hard limits
        left_pwm  = max(min(left_pwm, 1.0), -1.0)
        right_pwm = max(min(right_pwm, 1.0), -1.0)

        # Deadzone to stop motor humming
        if abs(v) < 0.001 and abs(w) < 0.001:
            left_pwm = 0.0
            right_pwm = 0.0

        line = f"CMD {left_pwm:.2f} {right_pwm:.2f}\n"
        self.get_logger().info(f"Sending to ESP32 : {line.strip()}")
        
        with self.ser_lock:
            self.ser.write(line.encode('utf-8'))

    def read_serial_loop(self):
        """ Continuously reads Odom ticks from the microcontroller """
        while rclpy.ok():
            with self.ser_lock:
                if self.ser.in_waiting > 0:
                    try:
                        line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                        if line.startswith("ODOM"):
                            parts = line.split()
                            if len(parts) == 3:
                                self.curr_l = int(parts[1])
                                self.curr_r = int(parts[2])
                                self.update_odometry()
                    except Exception as e:
                        self.get_logger().warn(f"Serial Read Error: {e}")

    def update_odometry(self):
        if self.prev_l is None:
            self.prev_l = self.curr_l
            self.prev_r = self.curr_r
            return

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        # Delta Ticks
        dl_ticks = self.curr_l - self.prev_l
        dr_ticks = self.curr_r - self.prev_r
        self.prev_l = self.curr_l
        self.prev_r = self.curr_r

        # Distance per tick = (2 * PI * R) / Ticks_Per_Rev
        dist_per_tick_r = (2.0 * math.pi * self.wheel_radius) / 275.0 # Adjust if right wheel has different ticks/rev
        dist_per_tick_l= (2.0 * math.pi * self.wheel_radius) / 275.0 # Adjust if left wheel has different ticks/rev
        d_l = dl_ticks * dist_per_tick_l
        d_r = dr_ticks * dist_per_tick_r
        
        d_center = (d_r + d_l) / 2.0
        d_theta  = (d_r - d_l) / self.wheel_base

        # Update Pose
        self.theta += d_theta
        self.x += d_center * math.cos(self.theta)
        self.y += d_center * math.sin(self.theta)

        # 1. Publish Odometry Message
        q = quaternion_from_euler(0.0, 0.0, self.theta)
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        
        self.odom_pub.publish(odom)

        # 2. Publish Transform (TF)
        t = TransformStamped()
        t.header = odom.header
        t.child_frame_id = odom.child_frame_id
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = AMRNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop motors before closing
        node.ser.write(b"CMD 0.00 0.00\n")
        node.ser.close()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
