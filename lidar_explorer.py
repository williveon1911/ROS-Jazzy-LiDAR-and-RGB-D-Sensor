#!/usr/bin/env python3
"""
LiDAR Data Explorer
Purpose: Understand what's in LiDAR data and visualize it
This is educational - you'll see exactly what each field means

Location: ~/ros2_ws/src/ROS-Jazzy-LiDAR-and-RGB-D-Sensor/lidar_explorer.py
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import math


class LiDARExplorer(Node):
    """
    Subscribes to LiDAR data and prints detailed information
    This helps you understand the LaserScan message structure
    """
    
    def __init__(self):
        super().__init__('lidar_explorer')
        
        # Subscribe to LiDAR scan topic
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10  # Queue size
        )
        
        # Track scan count
        self.scan_count = 0
        self.get_logger().info("🔍 LiDAR Explorer started - listening to /scan topic")
        
    def lidar_callback(self, msg: LaserScan):
        """
        Called every time a new LiDAR scan arrives
        Let's examine what's inside
        """
        self.scan_count += 1
        
        # Only print every 10 scans to avoid spam
        if self.scan_count % 10 != 0:
            return
        
        # ===== UNDERSTANDING THE LaserScan MESSAGE =====
        
        # 1. BASIC INFORMATION
        print("\n" + "="*60)
        print(f"📊 SCAN #{self.scan_count}")
        print("="*60)
        
        print(f"\n📍 Frame ID: {msg.header.frame_id}")
        print(f"⏱️  Timestamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}")
        
        # 2. SCAN GEOMETRY (How the scanner is configured)
        print(f"\n🎯 Scan Geometry:")
        print(f"   Angle range: {math.degrees(msg.angle_min):.1f}° to {math.degrees(msg.angle_max):.1f}°")
        print(f"   Angular increment: {math.degrees(msg.angle_increment):.2f}° between each reading")
        print(f"   Range: {msg.range_min:.2f}m (min) to {msg.range_max:.2f}m (max)")
        print(f"   Number of readings: {len(msg.ranges)}")
        
        # 3. RANGE DATA (The actual distance measurements)
        ranges = np.array(msg.ranges)
        print(f"\n📏 Range Statistics:")
        print(f"   Valid readings: {np.sum(~np.isinf(ranges))}/{len(ranges)}")
        print(f"   Invalid (inf/nan): {np.sum(np.isinf(ranges) | np.isnan(ranges))}")
        
        # Filter out invalid readings
        valid_ranges = ranges[~np.isinf(ranges)]
        valid_ranges = valid_ranges[~np.isnan(valid_ranges)]
        valid_ranges = valid_ranges[valid_ranges > msg.range_min]
        
        if len(valid_ranges) > 0:
            print(f"   Closest object: {np.min(valid_ranges):.3f}m")
            print(f"   Farthest object: {np.max(valid_ranges):.3f}m")
            print(f"   Average distance: {np.mean(valid_ranges):.3f}m")
        
        # 4. ANALYZE WHERE OBSTACLES ARE
        print(f"\n🧭 Obstacle Distribution:")
        self._analyze_obstacle_regions(msg)
        
        # 5. EXAMPLE: Convert polar to Cartesian coordinates
        print(f"\n📐 Converting to Cartesian Coordinates:")
        self._convert_to_cartesian(msg)
        
    def _analyze_obstacle_regions(self, msg: LaserScan):
        """
        Divide the scan into regions and show obstacle presence
        This teaches you about angular sectors
        """
        ranges = np.array(msg.ranges)
        
        # Divide into 8 regions (8 directions: N, NE, E, SE, S, SW, W, NW)
        num_regions = 8
        region_size = len(ranges) // num_regions
        region_names = ["Front (0°)", "Front-Left (45°)", "Left (90°)", 
                       "Back-Left (135°)", "Back (180°)", "Back-Right (225°)",
                       "Right (270°)", "Front-Right (315°)"]
        
        for i in range(num_regions):
            start_idx = i * region_size
            end_idx = (i + 1) * region_size
            region_ranges = ranges[start_idx:end_idx]
            
            # Filter valid readings
            valid = region_ranges[~np.isinf(region_ranges)]
            valid = valid[~np.isnan(valid)]
            valid = valid[valid > 0.1]
            
            if len(valid) > 0:
                min_dist = np.min(valid)
                avg_dist = np.mean(valid)
                print(f"   {region_names[i]}: min={min_dist:.2f}m, avg={avg_dist:.2f}m")
            else:
                print(f"   {region_names[i]}: No obstacles detected")
    
    def _convert_to_cartesian(self, msg: LaserScan):
        """
        Show how to convert LiDAR polar coordinates to (x, y) points
        This is the FOUNDATION of SLAM - converting sensor data to useful coords
        """
        ranges = np.array(msg.ranges)
        
        # Create angles for each range reading
        angles = np.arange(len(ranges)) * msg.angle_increment + msg.angle_min
        
        # Convert polar (angle, range) to Cartesian (x, y)
        # x = distance * cos(angle)
        # y = distance * sin(angle)
        x_points = ranges * np.cos(angles)
        y_points = ranges * np.sin(angles)
        
        # Filter out invalid points
        valid_mask = ~np.isinf(ranges) & ~np.isnan(ranges) & (ranges > msg.range_min)
        x_valid = x_points[valid_mask]
        y_valid = y_points[valid_mask]
        
        print(f"   Converted {len(x_valid)} readings to (x, y) points")
        
        # Show a few example points
        if len(x_valid) > 0:
            print(f"   Sample points:")
            sample_indices = [0, len(x_valid)//4, len(x_valid)//2, 3*len(x_valid)//4, -1]
            for idx in sample_indices:
                if idx < len(x_valid):
                    print(f"      Point: x={x_valid[idx]:.3f}m, y={y_valid[idx]:.3f}m")


def main(args=None):
    rclpy.init(args=args)
    explorer = LiDARExplorer()
    
    try:
        print("\n🚀 LiDAR Explorer is running...")
        print("📡 Waiting for LiDAR data on /scan topic...")
        print("(Press Ctrl+C to stop)\n")
        
        rclpy.spin(explorer)
    except KeyboardInterrupt:
        print("\n\n👋 LiDAR Explorer stopped")
    finally:
        explorer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()