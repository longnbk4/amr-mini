#!/usr/bin/env python3

import sys
import time
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
from ament_index_python.packages import get_package_share_directory
import os
import xacro


def main():
    # Get command line arguments
    args = sys.argv[1:] if sys.argv[1:] else []
    
    # Filter out ros-specific args
    clean_args = [arg for arg in args if not arg.startswith('--ros-args')]
    
    rclpy.init(args=sys.argv)
    
    # Parse arguments using standard argparse
    import argparse
    parser = argparse.ArgumentParser(description='Spawn robot in Gazebo')
    parser.add_argument('--x', type=float, default=0.0, help='X position')
    parser.add_argument('--y', type=float, default=0.0, help='Y position')
    parser.add_argument('--z', type=float, default=0.1, help='Z position')
    parser.add_argument('--entity', type=str, default='diff_bot', help='Entity name')
    parser.add_argument('--timeout', type=int, default=30, help='Timeout for service in seconds')
    
    parsed_args = parser.parse_args(clean_args)
    
    node = rclpy.create_node('spawn_robot')
    
    client = node.create_client(SpawnEntity, '/spawn_entity')
    
    # Wait for service with timeout
    timeout = parsed_args.timeout
    retry_count = 0
    while not client.wait_for_service(timeout_sec=1.0):
        retry_count += 1
        if retry_count >= timeout:
            node.get_logger().error(f'Timeout waiting for /spawn_entity service after {timeout} seconds')
            node.destroy_node()
            rclpy.shutdown()
            return
        node.get_logger().info(f'Waiting for /spawn_entity service... ({retry_count}/{timeout})')
    
    node.get_logger().info('Connected to /spawn_entity service')
    
    # Get robot_description from parameter
    package_name = 'robot_simulation'
    robot_description_path = os.path.join(
        get_package_share_directory(package_name),
        'urdf',
        'robot.urdf.xacro'
    )
    
    # Check if file exists
    if not os.path.exists(robot_description_path):
        node.get_logger().error(f'Robot description file not found: {robot_description_path}')
        node.destroy_node()
        rclpy.shutdown()
        return
    
    node.get_logger().info(f'Loading robot description from: {robot_description_path}')
    
    # Read the xacro file and process it
    try:
        robot_description = xacro.process(robot_description_path)
        node.get_logger().info('Successfully loaded robot_description')
    except Exception as e:
        node.get_logger().error(f'Failed to load robot_description: {e}')
        node.destroy_node()
        rclpy.shutdown()
        return
    
    # Validate robot_description is not empty
    if not robot_description or len(robot_description.strip()) == 0:
        node.get_logger().error('Robot description is empty after processing')
        node.destroy_node()
        rclpy.shutdown()
        return
    
    # Create spawn request
    request = SpawnEntity.Request()
    request.name = parsed_args.entity
    request.xml = robot_description
    request.initial_pose.position.x = float(parsed_args.x)
    request.initial_pose.position.y = float(parsed_args.y)
    request.initial_pose.position.z = float(parsed_args.z)
    
    node.get_logger().info(f'Spawning robot "{request.name}" at ({parsed_args.x}, {parsed_args.y}, {parsed_args.z})...')
    
    # Call the service with retry logic
    max_retries = 3
    for attempt in range(1, max_retries + 1):
        node.get_logger().info(f'Spawn attempt {attempt}/{max_retries}')
        
        future = client.call_async(request)
        
        # Wait for result with timeout
        start_time = time.time()
        while rclpy.ok():
            elapsed = time.time() - start_time
            if elapsed > 30.0:  # 30 second timeout for service call
                node.get_logger().error(f'Service call timed out after 30 seconds')
                break
            
            # Spin once to process callbacks
            rclpy.spin_once(node, timeout_sec=0.1)
            
            if future.done():
                break
        
        if future.result() is not None:
            if future.result().success:
                node.get_logger().info(f'Successfully spawned robot "{request.name}"!')
                node.destroy_node()
                rclpy.shutdown()
                return
            else:
                node.get_logger().error(f'Failed to spawn robot: {future.result().status_message}')
        else:
            node.get_logger().error(f'Service call failed (attempt {attempt}/{max_retries})')
        
        if attempt < max_retries:
            wait_time = 2.0
            node.get_logger().info(f'Retrying in {wait_time} seconds...')
            time.sleep(wait_time)
    
    node.get_logger().error(f'All {max_retries} spawn attempts failed')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)

