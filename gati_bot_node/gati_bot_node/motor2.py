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
            self.ser_lock = threading.Lock()
            self.get_logger().info(f"Connected to {serial_port}")
        except Exception as e:
            self.get_logger().error(f"Serial Error: {e}")
            exit(1)

        # --- ROBOT PARAMETERS ---
        self.wheel_radius  = 0.0325   # meters (65mm diameter wheel)
        self.ticks_per_rev = 275.0    # encoder ticks per full wheel revolution
        self.max_speed     = 0.5      # m/s — used to normalize PWM output

        # TWO separate wheel_base values — they serve different purposes:
        #
        # PHYSICAL wheel_base (motor kinematics):
        #   Real measured distance between the two wheel contact points.
        #   Used in cmd_vel_callback to split linear+angular velocity into
        #   individual left/right wheel speeds sent to ESP32.
        self.wheel_base_physical  = 0.134   # meters — DO NOT change this

        # EFFECTIVE wheel_base (odometry):
        #   Calibrated value that makes a real 360° spin = 360° in RViz.
        #   Always less than physical due to wheel slip/contact patch.
        #   Tune this: if RViz spin < real, decrease; if RViz > real, increase.
        #   Formula: new = current × (rviz_degrees / 360)
        self.wheel_base_odom = 0.134    # meters — tune this for accurate rotation

        # --- ODOMETRY STATE ---
        self.x     = 0.0
        self.y     = 0.0
        self.theta = 0.0

        self.prev_l    = None
        self.prev_r    = None
        self.last_time = self.get_clock().now()

        # Thread-safe tick buffer — written by serial thread, read by update_odometry
        self._tick_lock  = threading.Lock()
        self._new_ticks  = False
        self._tick_l     = 0
        self._tick_r     = 0

        # -- ROS2 SUBSCRIBER / PUBLISHERS ---
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.odom_pub      = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer to process odometry at fixed rate (20 Hz) — decoupled from serial rate
        self.create_timer(0.05, self.odometry_timer_callback)

        # Serial read thread
        threading.Thread(target=self.read_serial_loop, daemon=True).start()

    # -------------------------------------------------------------------------
    # CMD_VEL → ESP32
    # Uses physical wheel_base to correctly split velocity commands
    # -------------------------------------------------------------------------
    def cmd_vel_callback(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z

        # Differential drive kinematics — physical wheel_base used here
        left_vel  = v - (w * self.wheel_base_physical / 2.0)
        right_vel = v + (w * self.wheel_base_physical / 2.0)

        # Normalize to [-1.0, 1.0] PWM range
        left_pwm  = max(min(left_vel  / self.max_speed, 1.0), -1.0)
        right_pwm = max(min(right_vel / self.max_speed, 1.0), -1.0)

        # Deadzone — prevents motor humming at rest
        if abs(v) < 0.001 and abs(w) < 0.001:
            left_pwm  = 0.0
            right_pwm = 0.0

        line = f"CMD {left_pwm:.2f} {right_pwm:.2f}\n"
        self.get_logger().debug(f"Sending: {line.strip()}")

        with self.ser_lock:
            self.ser.write(line.encode('utf-8'))

    # -------------------------------------------------------------------------
    # SERIAL READ THREAD
    # Only writes to shared tick buffer — never calls update_odometry directly
    # -------------------------------------------------------------------------
    def read_serial_loop(self):
        while rclpy.ok():
            with self.ser_lock:
                if self.ser.in_waiting > 0:
                    try:
                        line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                        if line.startswith("ODOM"):
                            parts = line.split()
                            if len(parts) == 3:
                                with self._tick_lock:
                                    self._tick_l    = int(parts[1])
                                    self._tick_r    = int(parts[2])
                                    self._new_ticks = True
                    except Exception as e:
                        self.get_logger().warn(f"Serial Read Error: {e}")

    # -------------------------------------------------------------------------
    # ODOMETRY TIMER (20 Hz)
    # Reads tick buffer safely and computes odometry
    # -------------------------------------------------------------------------
    def odometry_timer_callback(self):
        with self._tick_lock:
            if not self._new_ticks:
                return
            curr_l = self._tick_l
            curr_r = self._tick_r
            self._new_ticks = False

        self.update_odometry(curr_l, curr_r)

    # -------------------------------------------------------------------------
    # ODOMETRY CALCULATION
    # -------------------------------------------------------------------------
    def update_odometry(self, curr_l: int, curr_r: int):
        if self.prev_l is None:
            self.prev_l = curr_l
            self.prev_r = curr_r
            return

        now = self.get_clock().now()
        dt  = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        # Tick deltas — handles wrap-around for 16-bit counters from ESP32
        dl_ticks = self._wrap_ticks(curr_l - self.prev_l)
        dr_ticks = self._wrap_ticks(curr_r - self.prev_r)
        self.prev_l = curr_l
        self.prev_r = curr_r

        # Convert ticks to distance
        dist_per_tick = (2.0 * math.pi * self.wheel_radius) / self.ticks_per_rev
        d_l = dl_ticks * dist_per_tick
        d_r = dr_ticks * dist_per_tick

        # Differential drive odometry — effective wheel_base used here
        d_center = (d_r + d_l) / 2.0
        d_theta  = (d_r - d_l) / self.wheel_base_odom

        # Velocities for twist field (required by Nav2)
        vx = d_center / dt if dt > 0 else 0.0
        vz = d_theta  / dt if dt > 0 else 0.0

        # Update pose
        self.theta += d_theta
        self.x     += d_center * math.cos(self.theta)
        self.y     += d_center * math.sin(self.theta)

        q = quaternion_from_euler(0.0, 0.0, self.theta)

        # -----------------------------------------------------------------
        # Publish Odometry message
        # Covariance matrices are required by Nav2 AMCL and EKF nodes.
        # Diagonal values represent uncertainty in [x, y, z, roll, pitch, yaw].
        # Higher value = less confident. Tune if localization drifts.
        # -----------------------------------------------------------------
        odom                 = Odometry()
        odom.header.stamp    = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_footprint'

        odom.pose.pose.position.x    = self.x
        odom.pose.pose.position.y    = self.y
        odom.pose.pose.position.z    = 0.0
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        # Pose covariance [x, y, z, roll, pitch, yaw] — 6x6 row-major
        odom.pose.covariance = [
            0.001,  0.0,   0.0,  0.0,  0.0,  0.0,
            0.0,   0.001,  0.0,  0.0,  0.0,  0.0,
            0.0,   0.0,   1e-9,  0.0,  0.0,  0.0,
            0.0,   0.0,   0.0,  1e-9,  0.0,  0.0,
            0.0,   0.0,   0.0,  0.0,  1e-9,  0.0,
            0.0,   0.0,   0.0,  0.0,  0.0,   0.01
        ]

        # Twist (velocity) — required by Nav2 controller_server
        odom.twist.twist.linear.x  = vx
        odom.twist.twist.angular.z = vz

        # Twist covariance
        odom.twist.covariance = [
            0.001,  0.0,   0.0,  0.0,  0.0,  0.0,
            0.0,   0.001,  0.0,  0.0,  0.0,  0.0,
            0.0,   0.0,   1e-9,  0.0,  0.0,  0.0,
            0.0,   0.0,   0.0,  1e-9,  0.0,  0.0,
            0.0,   0.0,   0.0,  0.0,  1e-9,  0.0,
            0.0,   0.0,   0.0,  0.0,  0.0,   0.01
        ]

        self.odom_pub.publish(odom)

        # -----------------------------------------------------------------
        # Publish TF: odom → base_footprint
        # -----------------------------------------------------------------
        t                        = TransformStamped()
        t.header.stamp           = now.to_msg()
        t.header.frame_id        = 'odom'
        t.child_frame_id         = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x   = q[0]
        t.transform.rotation.y   = q[1]
        t.transform.rotation.z   = q[2]
        t.transform.rotation.w   = q[3]

        self.tf_broadcaster.sendTransform(t)

    # -------------------------------------------------------------------------
    # Tick wrap-around handler
    # Handles ESP32 counter overflow (e.g. 16-bit: 0→65535 wraps to small delta)
    # -------------------------------------------------------------------------
    @staticmethod
    def _wrap_ticks(delta: int, max_ticks: int = 32768) -> int:
        if delta > max_ticks:
            delta -= 2 * max_ticks
        elif delta < -max_ticks:
            delta += 2 * max_ticks
        return delta


def main():
    rclpy.init()
    node = AMRNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.ser.write(b"CMD 0.00 0.00\n")
        node.ser.close()
        rclpy.shutdown()

if __name__ == '__main__':
    main()