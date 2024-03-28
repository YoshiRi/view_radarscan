#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from radar_msgs.msg import RadarScan
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from rclpy.qos import QoSProfile, ReliabilityPolicy
import math

QOS_PROFILE = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)



class RadarScanVisualizer(Node):
    def __init__(self):
        super().__init__('radar_scan_visualizer')
        self.subscription = self.create_subscription(
            RadarScan,
            'radar_scan',
            self.radar_scan_callback,
            QOS_PROFILE)
        self.marker_pub = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)
        # ros params
        self.declare_parameter('pointcloud_size', 1.0)
        self.declare_parameter('pointcloud_color', 'intensity')
        self.declare_parameter('output_frame', '')
        self.declare_parameter('max_height', 2.0)
        self.declare_parameter('min_height', -1.0)
        self.declare_parameter('height_filter', False)
        self.pointcloud_size = self.get_parameter('pointcloud_size').value
        self.pointcloud_color = self.get_parameter('pointcloud_color').value
        self.output_frame = self.get_parameter('output_frame').value
        self.height_filter = self.get_parameter('height_filter').value
        self.min_height = self.get_parameter('min_height').value
        self.max_height = self.get_parameter('max_height').value
        self.get_logger().info("pointcloud size is " + str(self.pointcloud_size))

    def radar_scan_callback(self, msg):
        marker_array = MarkerArray()
        for i, target in enumerate(msg.returns):
            # calculate xyz
            x = target.range * math.cos(target.azimuth)  # Convert polar to Cartesian coordinates
            y = target.range * math.sin(target.azimuth)
            z = target.range * math.sin(target.elevation)  # Assuming radar targets are in 2D plane
            if self.height_filter:
                if z > self.max_height or z < self.min_height:
                    continue

            # Create point marker
            point_marker = Marker()
            point_marker.header.frame_id = self.output_frame if self.output_frame else msg.header.frame_id
            point_marker.header.stamp = self.get_clock().now().to_msg()
            point_marker.ns = "radar_targets"
            point_marker.id = i
            point_marker.type = Marker.SPHERE
            point_marker.action = Marker.ADD
            point_marker.pose.position.x = x
            point_marker.pose.position.y = y
            point_marker.pose.position.z = z  # Assuming radar targets are in 2D plane
            point_marker.scale.x = self.pointcloud_size  # Adjust size as needed
            point_marker.scale.y = self.pointcloud_size
            point_marker.scale.z = self.pointcloud_size
            point_marker.color.a = 1.0  # Don't forget to set the alpha!
            point_marker.color.r = 1.0
            point_marker.color.g = 0.0
            point_marker.color.b = 0.0
            
            # Create velocity arrow marker
            velocity_marker = Marker()
            velocity_marker.header.frame_id = self.output_frame if self.output_frame else msg.header.frame_id
            velocity_marker.header.stamp = self.get_clock().now().to_msg()
            velocity_marker.ns = "radar_velocity"
            velocity_marker.id = i
            velocity_marker.type = Marker.ARROW
            velocity_marker.action = Marker.ADD
            velocity_marker.pose.position.x = x
            velocity_marker.pose.position.y = y
            velocity_marker.pose.position.z = z  # Assuming radar targets are in 2D plane
            # quaternion from yaw angle
            azimuth = target.azimuth
            velocity_marker.pose.orientation.x = 0.0
            velocity_marker.pose.orientation.y = 0.0
            velocity_marker.pose.orientation.z = math.sin(azimuth / 2)
            velocity_marker.pose.orientation.w = math.cos(azimuth / 2)
            # The length of the arrow represents the speed
            speed = target.doppler_velocity
            velocity_marker.scale.x = speed  # Arrow length
            velocity_marker.scale.y = 0.1  # Arrow width
            velocity_marker.scale.z = 0.1  # Arrow height
            # change intensity
            amplitude = target.amplitude
            # TODO: amplitude to rgb
            velocity_marker.color.a = 1.0
            velocity_marker.color.r = 0.0
            velocity_marker.color.g = 1.0
            velocity_marker.color.b = 0.0
            
            marker_array.markers.append(point_marker)
            marker_array.markers.append(velocity_marker)

        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = RadarScanVisualizer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
