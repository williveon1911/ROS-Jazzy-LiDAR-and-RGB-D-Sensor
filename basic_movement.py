#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import time

class BasicMovementNode(Node):
    def __init__(self):
        super().__init__('basic_movement')
        self.cmd_vel_pub_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.rate = self.create_rate(10)  # 10 Hz

        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        self.get_logger().info("✅ Basic Movement Node Started")
        self.get_logger().info(f"📢 Publishing to topic: {self.cmd_vel_pub_.topic_name}")
    
    def stop_robot(self):
        stop_msg = Twist()
        self.cmd_vel_pub_.publish(stop_msg)
        self.get_logger().info("🛑 Stop command published")

    def move_forward(self, speed=0.2, duration=2.0):
        self.get_logger().info(f"🚗 Starting move_forward: speed={speed}, duration={duration}")
        
        move_msg = Twist()
        move_msg.linear.x = speed
        
        start_time = self.get_clock().now()
        current_time = self.get_clock().now()
        elapsed = 0.0
        
        # Simple counter to track publishing
        publish_count = 0
        
        while elapsed < duration:
            # Publish the message
            self.cmd_vel_pub_.publish(move_msg)
            publish_count += 1
            
            # Log every 10th publish to avoid spam
            if publish_count % 10 == 0:
                self.get_logger().info(f"📤 Published {publish_count} times: linear.x={move_msg.linear.x}")
            
            # Wait for next cycle
            time.sleep(0.1)  # Sleep for 100ms
            
            # Update elapsed time
            current_time = self.get_clock().now()
            elapsed = (current_time - start_time).nanoseconds / 1e9
        
        self.get_logger().info(f"✅ Finished moving forward - Published {publish_count} times")
        self.stop_robot()
    
    def twist_90_degrees(self, angular_speed=0.5, direction="right"):
        self.get_logger().info(f"🔄 Starting twist: speed={angular_speed}, direction={direction}")
        
        twist_msg = Twist()
        
        # Set direction
        if direction == "left":
            twist_msg.angular.z = abs(angular_speed)  # Positive = left
            self.get_logger().info("↪️ Turning LEFT")
        else:  # right
            twist_msg.angular.z = -abs(angular_speed)  # Negative = right
            self.get_logger().info("↩️ Turning RIGHT")

        # Calculate time for 90 degrees
        duration = (math.pi / 2) / abs(angular_speed)
        self.get_logger().info(f"⏱️ Will turn for {duration:.2f} seconds")
        
        start_time = self.get_clock().now()
        current_time = self.get_clock().now()
        elapsed = 0.0
        publish_count = 0
        
        while elapsed < duration:
            # Publish the message
            self.cmd_vel_pub_.publish(twist_msg)
            publish_count += 1
            
            # Log every 10th publish
            if publish_count % 10 == 0:
                self.get_logger().info(f"📤 Published {publish_count} times: angular.z={twist_msg.angular.z:.2f}")
            
            # Wait for next cycle
            time.sleep(0.1)  # Sleep for 100ms
            
            # Update elapsed time
            current_time = self.get_clock().now()
            elapsed = (current_time - start_time).nanoseconds / 1e9
        
        self.get_logger().info(f"✅ Finished turning - Published {publish_count} times")
        self.stop_robot()

class SquarePathNode(BasicMovementNode):
    def __init__(self):
        super().__init__()
        self.get_logger().info("⬜ Square Path Node Ready")

    def execute_square_path(self, side_length=0.5, speed=0.2):
        # Calculate duration for each side
        side_duration = side_length / speed
        self.get_logger().info(f"📐 Side duration: {side_duration:.2f} seconds")
        
        self.get_logger().info(f'🚀 STARTING SQUARE PATH: {side_length}m sides at {speed} m/s')
        
        for i in range(4):
            self.get_logger().info(f'📏 SIDE {i+1}/4')
            self.move_forward(speed=speed, duration=side_duration)
            
            if i < 3:  # Turn after first 3 sides
                self.get_logger().info(f'↪️ CORNER {i+1}/4')
                self.twist_90_degrees(angular_speed=0.5, direction="right")
        
        self.get_logger().info('✅ SQUARE PATH COMPLETE!')

def main(args=None):
    rclpy.init(args=args)
    
    print("🎮 Starting square movement demo...")
    
    # Create and run the square path
    square_mover = SquarePathNode()
    square_mover.execute_square_path(side_length=0.5, speed=0.2)
    
    print("👋 Shutting down...")
    
    # Clean shutdown
    square_mover.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()