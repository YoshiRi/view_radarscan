#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from radar_msgs.msg import RadarScan
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from rclpy.qos import QoSProfile, ReliabilityPolicy
import math

QOS_PROFILE = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

class RadarScanConverter:
    def __init__(self, pointcloud_size, pointcloud_color, output_frame, height_filter, min_height, max_height):
        self.pointcloud_size = pointcloud_size
        self.pointcloud_color = pointcloud_color
        self.output_frame = output_frame
        self.height_filter = height_filter
        self.min_height = min_height
        self.max_height = max_height

    def convert_scan_to_markers(self, msg):
        marker_array = MarkerArray()
        for i, target in enumerate(msg.returns):
            x, y, z = self.calculate_xyz(target)
            if self.should_filter_height(z):
                continue
            point_marker = self.create_point_marker(i, msg, x, y, z)
            velocity_marker = self.create_velocity_marker(i, msg, target, x, y, z)
            marker_array.markers.append(point_marker)
            marker_array.markers.append(velocity_marker)
        return marker_array

    def calculate_xyz(self, target):
        x = target.range * math.cos(target.azimuth)
        y = target.range * math.sin(target.azimuth)
        z = target.range * math.sin(target.elevation)
        return x, y, z

    def should_filter_height(self, z):
        return self.height_filter and (z > self.max_height or z < self.min_height)

    def create_point_marker(self, marker_id, msg, x, y, z):
        marker = Marker()
        marker.header.frame_id = self.output_frame if self.output_frame else msg.header.frame_id
        marker.header.stamp = msg.header.stamp
        marker.ns = "radar_targets"
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.scale.x = marker.scale.y = marker.scale.z = self.pointcloud_size
        marker.color.a = 1.0
        marker.color.r = 1.0  # Example: Static color setting, can be based on amplitude
        marker.color.g = 1.0
        marker.color.b = 1.0
        return marker
    
    def calc_color_from_intensity(self, intensity:float) -> tuple[float, float, float]:
        # Example: Convert intensity to RGB color
        r = 1.0 - intensity
        g = 1.0
        b = 1.0
        return r, g, b

    def create_velocity_marker(self, marker_id, msg, target, x, y, z):
        azimuth = target.azimuth
        speed = target.doppler_velocity
        marker = Marker()
        marker.header.frame_id = self.output_frame if self.output_frame else msg.header.frame_id
        marker.header.stamp = msg.header.stamp
        marker.ns = "radar_velocity"
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        # marker.pose.orientation.z = math.sin(azimuth / 2)
        # marker.pose.orientation.w = math.cos(azimuth / 2)
        marker.scale.x = speed
        marker.scale.y = marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        return marker

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
        self.load_parameters()
        self.converter = RadarScanConverter(
            self.pointcloud_size, self.pointcloud_color, self.output_frame,
            self.height_filter, self.min_height, self.max_height)

    def load_parameters(self):
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

    def radar_scan_callback(self, msg):
        marker_array = self.converter.convert_scan_to_markers(msg)
        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = RadarScanVisualizer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
