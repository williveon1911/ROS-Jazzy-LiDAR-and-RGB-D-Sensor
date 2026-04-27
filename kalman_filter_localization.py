#!/usr/bin/env python3
"""
Kalman Filter Localization
==========================
Implements a 2D Constant Velocity Kalman Filter for multi-sensor fusion pose
estimation, combining wheel odometry and LiDAR range data.

State vector  x = [x, y, vx, vy, theta]ᵀ  (5 × 1)
  x, y    – robot position in the world frame (meters)
  vx, vy  – robot velocity in the world frame (m/s)
  theta   – robot heading angle (radians)

Kalman Filter equations (linear case)
--------------------------------------
  Predict:
    x̂⁻ₖ  = F · x̂ₖ₋₁          (state prediction)
    P⁻ₖ   = F · Pₖ₋₁ · Fᵀ + Q  (covariance prediction)

  Update (one measurement source at a time):
    K   = P⁻ · Hᵀ · (H · P⁻ · Hᵀ + R)⁻¹  (Kalman gain)
    x̂   = x̂⁻ + K · (z − H · x̂⁻)          (state update)
    P   = (I − K · H) · P⁻                  (covariance update)

Sensor fusion strategy
-----------------------
  • /odom  (nav_msgs/Odometry)     → position (x, y, theta) update
  • /scan  (sensor_msgs/LaserScan) → forward velocity correction via
                                     consecutive-scan range-delta

Publications
------------
  • /estimated_pose      (geometry_msgs/PoseWithCovarianceStamped)
  • /estimated_velocity  (geometry_msgs/TwistWithCovarianceStamped)

Location: ~/ros2_ws/src/ROS-Jazzy-LiDAR-and-RGB-D-Sensor/kalman_filter_localization.py

Usage
-----
  ros2 run my_bot kalman_filter_localization
  # or directly:
  python3 kalman_filter_localization.py
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


# ---------------------------------------------------------------------------
# Configuration – tune these to match your robot's characteristics
# ---------------------------------------------------------------------------

# Process noise – how much we trust the constant-velocity motion model.
# Larger values = more uncertainty in the model = filter trusts measurements more.
PROCESS_NOISE_POS   = 0.01   # (m²/s²)  noise on x, y
PROCESS_NOISE_VEL   = 0.10   # (m²/s⁴)  noise on vx, vy
PROCESS_NOISE_THETA = 0.005  # (rad²/s²) noise on heading

# Odometry measurement noise – how noisy your wheel odometry is.
ODOM_NOISE_POS   = 0.05   # (m²)    position measurement noise
ODOM_NOISE_THETA = 0.02   # (rad²)  heading measurement noise

# LiDAR measurement noise – how noisy the forward-range velocity estimate is.
LIDAR_NOISE_VEL = 0.20  # (m²/s²) forward-velocity measurement noise

# Angular half-width (radians) of the forward sector used for LiDAR velocity.
FORWARD_SECTOR_HALF_WIDTH = math.radians(30)  # ±30° from robot heading
LIDAR_LOG_INTERVAL = 20   # Log a LiDAR update every N scans


# ---------------------------------------------------------------------------
# KalmanFilterLocalization node
# ---------------------------------------------------------------------------

class KalmanFilterLocalization(Node):
    """
    ROS 2 node that fuses LiDAR and odometry via a 2D Kalman Filter to
    produce smooth, uncertainty-aware pose and velocity estimates.
    """

    def __init__(self):
        super().__init__('kalman_filter_localization')

        # ----------------------------------------------------------------
        # Kalman Filter state
        # ----------------------------------------------------------------

        # State estimate: x = [x, y, vx, vy, theta]ᵀ
        self._x = np.zeros((5, 1))

        # State covariance P – initialised with large uncertainty
        self._P = np.diag([1.0, 1.0, 1.0, 1.0, 1.0])

        # State transition matrix F (constant velocity model, filled in at
        # each prediction step once dt is known)
        self._F = np.eye(5)

        # Process noise covariance Q
        #
        #   Q = diag(σ²_x, σ²_y, σ²_vx, σ²_vy, σ²_θ)
        #
        # This captures model uncertainty: vibrations, unmodelled accelerations,
        # terrain irregularities, etc.
        self._Q = np.diag([
            PROCESS_NOISE_POS,
            PROCESS_NOISE_POS,
            PROCESS_NOISE_VEL,
            PROCESS_NOISE_VEL,
            PROCESS_NOISE_THETA,
        ])

        # Odometry measurement matrix H_odom – maps state → [x, y, theta]
        #
        #   z_odom = H_odom · x  +  noise
        #   z_odom = [x, y, theta]ᵀ
        self._H_odom = np.array([
            [1, 0, 0, 0, 0],   # x
            [0, 1, 0, 0, 0],   # y
            [0, 0, 0, 0, 1],   # theta
        ])

        # Odometry measurement noise covariance R_odom
        self._R_odom = np.diag([
            ODOM_NOISE_POS,
            ODOM_NOISE_POS,
            ODOM_NOISE_THETA,
        ])

        # LiDAR measurement matrix H_lidar – maps state → [vx]
        #
        #   The forward-range derivative gives us an estimate of forward
        #   velocity (Δrange/Δt ≈ −vx in the robot frame).  Projecting onto
        #   world-frame vx gives an indirect velocity measurement.
        self._H_lidar = np.array([
            [0, 0, 1, 0, 0],   # vx
        ])

        # LiDAR measurement noise covariance R_lidar
        self._R_lidar = np.array([[LIDAR_NOISE_VEL]])

        # ----------------------------------------------------------------
        # Internal bookkeeping
        # ----------------------------------------------------------------

        self._last_predict_time = None      # ROS time of last prediction
        self._last_scan_min_range = None    # min forward range from previous scan
        self._last_scan_time = None         # ROS time of previous scan
        self._scan_count = 0

        # ----------------------------------------------------------------
        # Publishers
        # ----------------------------------------------------------------

        self._pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/estimated_pose',
            10,
        )
        self._vel_pub = self.create_publisher(
            TwistStamped,
            '/estimated_velocity',
            10,
        )

        # ----------------------------------------------------------------
        # Subscribers
        # ----------------------------------------------------------------

        self.create_subscription(Odometry, '/odom', self._odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self._scan_callback, 10)

        self.get_logger().info("🚀 Kalman Filter Localization node started")
        self.get_logger().info(
            f"📡 Subscribing to: /odom, /scan"
        )
        self.get_logger().info(
            f"📤 Publishing to:  /estimated_pose, /estimated_velocity"
        )
        self.get_logger().info(
            f"⚙️  Process noise   – pos: {PROCESS_NOISE_POS}, "
            f"vel: {PROCESS_NOISE_VEL}, theta: {PROCESS_NOISE_THETA}"
        )
        self.get_logger().info(
            f"⚙️  Odom meas noise – pos: {ODOM_NOISE_POS}, "
            f"theta: {ODOM_NOISE_THETA}"
        )
        self.get_logger().info(
            f"⚙️  LiDAR meas noise – vel: {LIDAR_NOISE_VEL}"
        )

    # ------------------------------------------------------------------
    # Prediction step
    # ------------------------------------------------------------------

    def _predict(self, current_time) -> float:
        """
        Kalman Filter prediction step using the constant-velocity motion model.

        Motion model
        ------------
          x(k)     = x(k−1) + vx(k−1) · dt
          y(k)     = y(k−1) + vy(k−1) · dt
          vx(k)    = vx(k−1)                  (velocity assumed constant)
          vy(k)    = vy(k−1)
          theta(k) = theta(k−1)

        State transition matrix F
        -------------------------
          F = | 1  0  dt  0   0 |
              | 0  1   0  dt  0 |
              | 0  0   1   0  0 |
              | 0  0   0   1  0 |
              | 0  0   0   0  1 |

        Returns the elapsed time dt (seconds).
        """
        if self._last_predict_time is None:
            self._last_predict_time = current_time
            return 0.0

        dt = (current_time - self._last_predict_time).nanoseconds * 1e-9
        self._last_predict_time = current_time

        if dt <= 0.0:
            return 0.0

        # Build state-transition matrix F for this dt
        #
        #   Constant-velocity kinematics:
        #     position  += velocity * dt
        #     velocity  unchanged
        #     heading   unchanged
        F = np.eye(5)
        F[0, 2] = dt   # x  += vx * dt
        F[1, 3] = dt   # y  += vy * dt
        self._F = F

        # ---- Predicted state: x̂⁻ = F · x̂ ----
        self._x = F @ self._x

        # ---- Predicted covariance: P⁻ = F · P · Fᵀ + Q ----
        self._P = F @ self._P @ F.T + self._Q

        return dt

    # ------------------------------------------------------------------
    # Generic measurement update step
    # ------------------------------------------------------------------

    def _update(self, z: np.ndarray, H: np.ndarray, R: np.ndarray):
        """
        Kalman Filter measurement update step.

        Parameters
        ----------
        z : (m, 1) measurement vector
        H : (m, 5) measurement matrix   (z = H · x + noise)
        R : (m, m) measurement noise covariance

        Kalman gain
        -----------
          S = H · P⁻ · Hᵀ + R          (innovation covariance)
          K = P⁻ · Hᵀ · S⁻¹            (Kalman gain)

        State update
        ------------
          innovation = z − H · x̂⁻      (measurement residual)
          x̂ = x̂⁻ + K · innovation

        Covariance update (Joseph form for numerical stability)
        -------------------------------------------------------
          I_KH = I − K · H
          P = I_KH · P⁻ · I_KHᵀ + K · R · Kᵀ
        """
        # Innovation (measurement residual)
        innovation = z - H @ self._x

        # Innovation covariance S
        S = H @ self._P @ H.T + R

        # Kalman gain K = P⁻ · Hᵀ · S⁻¹
        K = self._P @ H.T @ np.linalg.inv(S)

        # Updated state estimate
        self._x = self._x + K @ innovation

        # Normalise heading to [−π, π] after update
        self._x[4, 0] = self._normalise_angle(self._x[4, 0])

        # Joseph-form covariance update (numerically more stable than I−KH)
        n = self._P.shape[0]
        I_KH = np.eye(n) - K @ H
        self._P = I_KH @ self._P @ I_KH.T + K @ R @ K.T

    # ------------------------------------------------------------------
    # Odometry callback  → measurement update
    # ------------------------------------------------------------------

    def _odom_callback(self, msg: Odometry):
        """
        Receives wheel odometry and performs a Kalman measurement update.

        Measurement vector z_odom = [x_odom, y_odom, theta_odom]ᵀ

        The quaternion orientation is converted to a yaw angle (theta) and
        used together with the 2D position as the observation.
        """
        current_time = self.get_clock().now()

        # ---- Prediction step first ----
        self._predict(current_time)

        # ---- Extract odometry measurement ----
        x_odom = msg.pose.pose.position.x
        y_odom = msg.pose.pose.position.y

        # Convert quaternion → yaw (theta)
        q = msg.pose.pose.orientation
        theta_odom = self._quat_to_yaw(q.x, q.y, q.z, q.w)

        z_odom = np.array([[x_odom], [y_odom], [theta_odom]])

        # ---- Measurement update ----
        self._update(z_odom, self._H_odom, self._R_odom)

        # twist.linear.x is forward speed in robot frame; rotate to world frame
        # for debug logging only — computed only when debug logging is active
        self.get_logger().debug(
            f"[ODOM update]  z=({x_odom:.3f}, {y_odom:.3f}, {math.degrees(theta_odom):.1f}°) | "
            f"x̂=({self._x[0,0]:.3f}, {self._x[1,0]:.3f}) | "
            f"v_body={msg.twist.twist.linear.x:.3f} m/s → world "
            f"({msg.twist.twist.linear.x * math.cos(self._x[4, 0]):.3f}, "
            f"{msg.twist.twist.linear.x * math.sin(self._x[4, 0]):.3f})"
        )

        # ---- Publish estimates ----
        self._publish_estimates(msg.header.stamp)

    # ------------------------------------------------------------------
    # LiDAR callback  → velocity correction via range derivative
    # ------------------------------------------------------------------

    def _scan_callback(self, msg: LaserScan):
        """
        Receives a LiDAR scan and uses consecutive minimum-range readings in
        the forward sector to estimate the robot's forward velocity.

        Velocity estimate from range derivative
        ----------------------------------------
          Δrange = range_now − range_prev
          Δt     = time_now − time_prev
          v_lidar_estimate = −Δrange / Δt

        A negative Δrange (range decreasing) corresponds to the robot moving
        forward towards an obstacle, giving a positive forward velocity.

        This is then used as an additional measurement to update vx in the
        Kalman Filter, complementing the odometry-based update.
        """
        self._scan_count += 1
        current_time = self.get_clock().now()

        # ---- Extract minimum range in the forward sector ----
        ranges = np.array(msg.ranges, dtype=np.float64)
        angles = (
            np.arange(len(ranges)) * msg.angle_increment + msg.angle_min
        )

        # Forward sector: angles within ±FORWARD_SECTOR_HALF_WIDTH of 0°
        forward_mask = np.abs(angles) <= FORWARD_SECTOR_HALF_WIDTH
        forward_ranges = ranges[forward_mask]

        # Filter out invalid readings (inf, nan, below sensor minimum)
        valid_mask = (
            ~np.isinf(forward_ranges)
            & ~np.isnan(forward_ranges)
            & (forward_ranges > msg.range_min)
        )
        valid_forward = forward_ranges[valid_mask]

        if len(valid_forward) == 0:
            self._last_scan_min_range = None
            self._last_scan_time = current_time
            return

        min_forward_range = float(np.min(valid_forward))

        # ---- Compute range derivative to estimate forward velocity ----
        if self._last_scan_time is not None:
            dt_scan = (current_time - self._last_scan_time).nanoseconds * 1e-9

            if dt_scan > 0.0:
                # Δrange / Δt  gives rate of change of obstacle distance.
                # If the robot moves forward, range decreases → negative Δrange.
                # Forward velocity ≈ −Δrange / Δt
                delta_range = min_forward_range - self._last_scan_min_range
                vx_lidar = -delta_range / dt_scan

                # ---- Prediction step before LiDAR update ----
                self._predict(current_time)

                # ---- Measurement update using forward velocity ----
                z_lidar = np.array([[vx_lidar]])
                self._update(z_lidar, self._H_lidar, self._R_lidar)

                if self._scan_count % LIDAR_LOG_INTERVAL == 0:
                    self.get_logger().info(
                        f"🔭 [LiDAR update]  min_fwd_range={min_forward_range:.3f}m | "
                        f"Δrange={delta_range:.4f}m | dt={dt_scan:.3f}s | "
                        f"v_lidar={vx_lidar:.3f} m/s | "
                        f"x̂_vx={self._x[2,0]:.3f} m/s"
                    )

        self._last_scan_min_range = min_forward_range
        self._last_scan_time = current_time

    # ------------------------------------------------------------------
    # Publishing helpers
    # ------------------------------------------------------------------

    def _publish_estimates(self, stamp):
        """Publish the current state estimate to ROS 2 topics."""
        x_est, y_est, vx_est, vy_est, theta_est = self._x.flatten()

        # ---- Pose with covariance ----
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = 'odom'

        pose_msg.pose.pose.position.x = x_est
        pose_msg.pose.pose.position.y = y_est
        pose_msg.pose.pose.position.z = 0.0

        # Convert yaw back to quaternion for ROS 2 compatibility
        qx, qy, qz, qw = self._yaw_to_quat(theta_est)
        pose_msg.pose.pose.orientation.x = qx
        pose_msg.pose.pose.orientation.y = qy
        pose_msg.pose.pose.orientation.z = qz
        pose_msg.pose.pose.orientation.w = qw

        # Fill the 6×6 pose covariance (row-major, indices: x,y,z,roll,pitch,yaw)
        # We have covariance for (x, y, theta) → entries [0,0], [1,1], [5,5]
        cov6 = [0.0] * 36
        cov6[0]  = self._P[0, 0]   # var(x)
        cov6[1]  = self._P[0, 1]   # cov(x, y)
        cov6[6]  = self._P[1, 0]   # cov(y, x)
        cov6[7]  = self._P[1, 1]   # var(y)
        cov6[35] = self._P[4, 4]   # var(theta) → yaw slot
        pose_msg.pose.covariance = cov6

        self._pose_pub.publish(pose_msg)

        # ---- Velocity with covariance ----
        vel_msg = TwistWithCovarianceStamped()
        vel_msg.header.stamp = stamp
        vel_msg.header.frame_id = 'odom'

        vel_msg.twist.twist.linear.x = vx_est
        vel_msg.twist.twist.linear.y = vy_est
        vel_msg.twist.twist.linear.z = 0.0

        # 6×6 twist covariance (vx, vy, vz, wx, wy, wz)
        cov6_twist = [0.0] * 36
        cov6_twist[0]  = self._P[2, 2]   # var(vx)
        cov6_twist[1]  = self._P[2, 3]   # cov(vx, vy)
        cov6_twist[6]  = self._P[3, 2]   # cov(vy, vx)
        cov6_twist[7]  = self._P[3, 3]   # var(vy)
        vel_msg.twist.covariance = cov6_twist

        self._vel_pub.publish(vel_msg)

        self.get_logger().debug(
            f"📍 Estimated pose: x={x_est:.3f} m, y={y_est:.3f} m, "
            f"θ={math.degrees(theta_est):.1f}° | "
            f"v=({vx_est:.3f}, {vy_est:.3f}) m/s | "
            f"σ_x={math.sqrt(self._P[0,0]):.4f} m"
        )

    # ------------------------------------------------------------------
    # Utility helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _normalise_angle(angle: float) -> float:
        """Wrap an angle to the range [−π, π]."""
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def _quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
        """
        Extract yaw (rotation about Z) from a unit quaternion.

        yaw = atan2(2(qw·qz + qx·qy), 1 − 2(qy² + qz²))
        """
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _yaw_to_quat(yaw: float):
        """
        Convert a yaw angle to a quaternion (roll = pitch = 0).

        q = (0, 0, sin(yaw/2), cos(yaw/2))
        """
        half_yaw = yaw / 2.0
        return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = KalmanFilterLocalization()

    try:
        print("\n🤖 Kalman Filter Localization is running...")
        print("📡 Listening on /odom and /scan ...")
        print("📤 Publishing to /estimated_pose and /estimated_velocity")
        print("(Press Ctrl+C to stop)\n")

        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\n👋 Kalman Filter Localization stopped")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
