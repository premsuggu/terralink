#!/usr/bin/env python3
"""
Synthetic PointCloud2 Generator for Testing

Generates synthetic point clouds for testing elevation mapping without Gazebo.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import ros2_numpy as rnp
import math


class SyntheticPointCloudPublisher(Node):
    def __init__(self):
        super().__init__('synthetic_pointcloud_publisher')
        
        # Parameters
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('num_points', 10000)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('sensor_frame', 'camera_depth_optical_frame')
        self.declare_parameter('uav_height', 5.0)
        self.declare_parameter('ground_type', 'flat')
        
        self.publish_rate = self.get_parameter('publish_rate').value
        self.num_points = self.get_parameter('num_points').value
        self.map_frame = self.get_parameter('map_frame').value
        self.sensor_frame = self.get_parameter('sensor_frame').value
        self.uav_height = self.get_parameter('uav_height').value
        self.ground_type = self.get_parameter('ground_type').value
        
        # Publisher
        self.pc_pub = self.create_publisher(
            PointCloud2, '/terralink_uav/depth_camera/points', 
            QoSPresetProfiles.SENSOR_DATA.value
        )
        
        # Timer
        self.timer = self.create_timer(
            1.0 / self.publish_rate, self.timer_callback
        )
        
        self.time = 0.0
        self.get_logger().info(f'Synthetic PointCloud publisher started: {self.ground_type}')
    
    def timer_callback(self):
        # Check if context is still valid
        if not rclpy.ok():
            return
            
        # Generate synthetic point cloud
        points = self.generate_points()
        
        # Create PointCloud2 message
        msg = self.create_pointcloud2(points)
        
        # Check if publisher is still valid
        if rclpy.ok():
            self.pc_pub.publish(msg)
        
        self.time += 1.0 / self.publish_rate
    
    def generate_points(self):
        """Generate synthetic points based on ground type."""
        # Camera intrinsics (simulated)
        width, height = 640, 480
        fx, fy = 500.0, 500.0
        cx, cy = width / 2, height / 2
        
        # Generate random pixel coordinates
        u = np.random.uniform(0, width, self.num_points)
        v = np.random.uniform(0, height, self.num_points)
        
        # Depth based on ground type
        if self.ground_type == 'flat':
            # Flat ground at z=0, camera at height
            depth = self.uav_height / np.cos(np.arctan2(v - cy, fy))
            depth = np.clip(depth, 0.5, 20.0)
        elif self.ground_type == 'gaussian_bump':
            # Gaussian bump at center - iterative depth computation
            # Start with flat ground depth estimate
            depth = self.uav_height / np.cos(np.arctan2(v - cy, fy))
            depth = np.clip(depth, 0.5, 20.0)
            # Refine with ground height
            for _ in range(3):
                x_cam = (u - cx) * depth / fx
                y_cam = (v - cy) * depth / fy
                ground_z = 1.5 * np.exp(-(x_cam**2 + y_cam**2) / 8.0)  # 1.5m high bump
                depth = (self.uav_height - ground_z) / np.cos(np.arctan2(v - cy, fy))
                depth = np.clip(depth, 0.5, 20.0)
        else:
            depth = np.random.uniform(0.5, 20.0, self.num_points)
        
        # Convert to 3D points in sensor frame
        x = (u - cx) * depth / fx
        y = (v - cy) * depth / fy
        z = depth
        
        # Filter valid points (in front of camera)
        mask = z > 0
        points = np.column_stack([x[mask], y[mask], z[mask]]).astype(np.float32)
        
        return points
    
    def _make_pointcloud2(self, points_xyz, frame_id, stamp):
        """Create a PointCloud2 with xyz float32 fields from an (N,3) numpy array."""
        if points_xyz.ndim != 2 or points_xyz.shape[1] != 3:
            raise ValueError(f"points_xyz must have shape (N,3), got {points_xyz.shape}")
        msg = PointCloud2()
        msg.header.frame_id = frame_id
        msg.header.stamp = stamp
        msg.height = 1
        msg.width = int(points_xyz.shape[0])
        msg.is_bigendian = False
        msg.is_dense = True
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.data = np.asarray(points_xyz, dtype=np.float32).tobytes()
        return msg

    def create_pointcloud2(self, points):
        """Create PointCloud2 message from (N, 3) points."""
        stamp = self.get_clock().now().to_msg()
        return self._make_pointcloud2(points, self.sensor_frame, stamp)


def main(args=None):
    rclpy.init(args=args)
    node = SyntheticPointCloudPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except BaseException as e:
        if "ExternalShutdownException" not in str(type(e)):
            raise
    finally:
        try:
            node.destroy_node()
        except:
            pass
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()