#!/usr/bin/env python3
"""
OnRobot RG2/RG6 Gripper Usage Example

This example demonstrates how to control an OnRobot RG2/RG6 gripper using ROS2.
"""

import rclpy
from rclpy.node import Node
from onrobot_msgs.msg import RGStatus, RGCommand
import time


class OnRobotGripperExample(Node):
    def __init__(self):
        super().__init__('onrobot_gripper_example')
        
        # Publisher for sending commands to the gripper
        self.command_pub = self.create_publisher(
            RGCommand,
            'rg_command',
            10
        )
        
        # Subscriber for receiving gripper status
        self.status_sub = self.create_subscription(
            RGStatus,
            'rg_status',
            self.status_callback,
            10
        )
        
        self.current_status = None
        self.get_logger().info('OnRobot Gripper Example Node Started')
    
    def status_callback(self, msg):
        """Callback to receive gripper status"""
        self.current_status = msg
        
        # Log status (only on first receive)
        if not hasattr(self, 'status_logged'):
            self.get_logger().info(f'Gripper Connected - Serial: {msg.serial_number}')
            self.get_logger().info(f'Current Position: {msg.actual_width:.1f} mm')
            self.get_logger().info(f'Operating Range: {msg.min_width:.1f} ~ {msg.max_width:.1f} mm')
            self.status_logged = True
    
    def wait_for_status(self, timeout=5.0):
        """Wait for gripper status"""
        start_time = time.time()
        while self.current_status is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start_time > timeout:
                self.get_logger().error('Gripper status receive timeout')
                return False
        return True
    
    def wait_for_motion_complete(self, timeout=10.0):
        """Wait until gripper motion is complete"""
        start_time = time.time()
        while self.current_status and self.current_status.busy:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start_time > timeout:
                self.get_logger().error('Motion completion timeout')
                return False
        return True
    
    def grip(self, width_mm, force_n=100.0):
        """
        Grip an object with the gripper.
        
        Args:
            width_mm: Target width (mm)
            force_n: Target force (N) - RG2: 0-400N, RG6: 0-1200N
        """
        cmd = RGCommand()
        cmd.command = RGCommand.GRIP
        cmd.target_width = width_mm
        cmd.target_force = force_n
        
        self.get_logger().info(f'Sending grip command: width={width_mm}mm, force={force_n}N')
        self.command_pub.publish(cmd)
        
        # Wait for motion completion
        time.sleep(0.1)  # Brief wait for command to be sent
        if self.wait_for_motion_complete():
            if self.current_status.grip_detected:
                self.get_logger().info('Object grip detected!')
            else:
                self.get_logger().warn('No object grip detected')
    
    def open(self, width_mm=None):
        """
        Open the gripper.
        
        Args:
            width_mm: Target width (mm). Opens to maximum if None
        """
        if width_mm is None:
            width_mm = self.current_status.max_width if self.current_status else 110.0
        
        cmd = RGCommand()
        cmd.command = RGCommand.GRIP
        cmd.target_width = width_mm
        cmd.target_force = 40.0  # Open with low force
        
        self.get_logger().info(f'Opening gripper: {width_mm}mm')
        self.command_pub.publish(cmd)
        
        time.sleep(0.1)
        self.wait_for_motion_complete()
    
    def stop(self):
        """Immediately stop gripper motion."""
        cmd = RGCommand()
        cmd.command = RGCommand.STOP
        
        self.get_logger().info('Sending stop command')
        self.command_pub.publish(cmd)


def main():
    rclpy.init()
    
    node = OnRobotGripperExample()
    
    # Wait for gripper status reception
    node.get_logger().info('Waiting for gripper status...')
    if not node.wait_for_status():
        return
    
    try:
        # Example 1: Fully open gripper
        node.get_logger().info('\n=== Example 1: Opening Gripper ===')
        node.open()
        time.sleep(2.0)
        
        # Example 2: Grip at 50mm width (medium force)
        node.get_logger().info('\n=== Example 2: Grip at 50mm Width ===')
        node.grip(width_mm=50.0, force_n=100.0)
        time.sleep(2.0)
        
        # Example 3: Open again
        node.get_logger().info('\n=== Example 3: Opening Again ===')
        node.open()
        time.sleep(2.0)
        
        # Example 4: Grip small object (strong force)
        node.get_logger().info('\n=== Example 4: Grip at 30mm with Strong Force ===')
        node.grip(width_mm=30.0, force_n=200.0)
        time.sleep(2.0)
        
        # Example 5: Final open
        node.get_logger().info('\n=== Example 5: Final Opening ===')
        node.open()
        
        node.get_logger().info('\nAll examples completed!')
        
    except KeyboardInterrupt:
        node.get_logger().info('User interrupted')
        node.stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
