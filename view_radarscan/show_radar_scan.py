#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from radar_msgs.msg import RadarScan
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, QuaternionStamped
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy
import math
from collections import deque
from typing import Optional
import tf2_ros
from geometry_msgs.msg import Transform

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
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.scale.x = marker.scale.y = marker.scale.z = self.pointcloud_size
        marker.color.a = 1.0
        marker.color.r = 1.0  # Example: Static color setting, can be based on amplitude
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.lifetime = Duration(seconds=10.0).to_msg()
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
        marker.pose.orientation.z = math.sin(azimuth / 2)
        marker.pose.orientation.w = math.cos(azimuth / 2)
        marker.scale.x = speed
        marker.scale.y = marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.lifetime = Duration(seconds=10.0).to_msg()
        return marker

class EgomotionCancel:
    def __init__(self):
        self.accepable_threshold = 0.05 # 50ms
        # get static transform from base_link to sensor

    def cancel_ego_motion(self, stamp, marker_array, odom_queue, baselink_to_sensor_transform:Transform):
        # 1. get interpolated odometry
        interpolated_odom = self.interpolate_twist_odometry(stamp, odom_queue)
        if interpolated_odom is None:
            return marker_array
        # 2. calculate motion in the transformed frame
        v_ego = interpolated_odom.twist.twist.linear.x
        w_ego = interpolated_odom.twist.twist.angular.z
        # base_link to sensor
        x_b2s = baselink_to_sensor_transform.translation.x
        y_b2s = baselink_to_sensor_transform.translation.y
        yaw_b2s = 2 * math.atan2(baselink_to_sensor_transform.rotation.z, baselink_to_sensor_transform.rotation.w)
        
        # 3. cancel the motion within the azimuth range of radar scan
        for marker in marker_array.markers:
            if not marker.ns == "radar_velocity":
                continue
            # if marker is velocity
            azimuth = 2 * math.atan2(marker.pose.orientation.z, marker.pose.orientation.w)
            speed = marker.scale.x
            # cancel ego motion
            v_d = (v_ego - w_ego * y_b2s) * math.cos(azimuth+yaw_b2s) - w_ego * x_b2s * math.sin(azimuth+yaw_b2s)
            marker.scale.x = speed + v_d
        return marker_array
    

    def interpolate_twist_odometry(self, stamp, odom_queue):
        # interpolate odometry with specified timestamp
        # get odom pair with closest timestamp
        def get_time_diff_in_seconds(stamp1, stamp2):
            time1 = stamp1.sec + stamp1.nanosec / 1e9
            time2 = stamp2.sec + stamp2.nanosec / 1e9
            return time1 - time2

        for i in range(len(odom_queue)):
            if get_time_diff_in_seconds(odom_queue[i].header.stamp, stamp) > 0:
                break

        if i == 0:
            if abs(get_time_diff_in_seconds(odom_queue[i].header.stamp, stamp)) < self.accepable_threshold:
                return odom_queue[i]
            else:
                return None
        elif i == len(odom_queue):
            if abs(get_time_diff_in_seconds(odom_queue[i-1].header.stamp, stamp)) < self.accepable_threshold:
                return odom_queue[i-1]
            else:
                return None
        else:
            # interpolate odom
            t1 = get_time_diff_in_seconds(odom_queue[i].header.stamp, stamp)
            t2 = get_time_diff_in_seconds(odom_queue[i-1].header.stamp, stamp)
            a1 = t1 / (t1 + t2)
            a2 = t2 / (t1 + t2)
            interpolated_odom = Odometry()
            interpolated_odom.header.stamp = stamp
            interpolated_odom.header.frame_id = odom_queue[i].header.frame_id
            # interpolate twist only
            interpolated_odom.twist.twist.linear.x = a1 * odom_queue[i].twist.twist.linear.x + a2 * odom_queue[i-1].twist.twist.linear.x
            interpolated_odom.twist.twist.linear.y = a1 * odom_queue[i].twist.twist.linear.y + a2 * odom_queue[i-1].twist.twist.linear.y
            interpolated_odom.twist.twist.linear.z = a1 * odom_queue[i].twist.twist.linear.z + a2 * odom_queue[i-1].twist.twist.linear.z
            interpolated_odom.twist.twist.angular.x = a1 * odom_queue[i].twist.twist.angular.x + a2 * odom_queue[i-1].twist.twist.angular.x
            interpolated_odom.twist.twist.angular.y = a1 * odom_queue[i].twist.twist.angular.y + a2 * odom_queue[i-1].twist.twist.angular.y
            interpolated_odom.twist.twist.angular.z = a1 * odom_queue[i].twist.twist.angular.z + a2 * odom_queue[i-1].twist.twist.angular.z
            return interpolated_odom # twist only

class RadarScanVisualizer(Node):
    def __init__(self):
        super().__init__('radar_scan_visualizer')
        # radar scan subscriber
        self.subscription = self.create_subscription(
            RadarScan,
            'radar_scan',
            self.radar_scan_callback,
            QOS_PROFILE)
        # odom subscriber
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/localization/kinematic_state',
            self.odom_callback,
            QOS_PROFILE)
        # static tf listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.ego_motion_canceller = EgomotionCancel()

        self.sensor_frame:Optional[str] = None
        self.marker_pub = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)
        # ros params
        self.load_parameters()
        self.converter = RadarScanConverter(
            self.pointcloud_size, self.pointcloud_color, self.output_frame,
            self.height_filter, self.min_height, self.max_height)
        # length of odom que
        self.odom_queue = deque([], maxlen=10)

    def load_parameters(self):
        self.declare_parameter('pointcloud_size', 1.0)
        self.declare_parameter('pointcloud_color', 'intensity')
        self.declare_parameter('output_frame', '')
        self.declare_parameter('max_height', 2.0)
        self.declare_parameter('min_height', -1.0)
        self.declare_parameter('min_range', 80.0)
        self.declare_parameter('max_range', 300.0)
        self.declare_parameter('height_filter', False)
        self.declare_parameter('range_filter', False)
        self.declare_parameter('odom_cancel', True)
        self.pointcloud_size = self.get_parameter('pointcloud_size').value
        self.pointcloud_color = self.get_parameter('pointcloud_color').value
        self.output_frame = self.get_parameter('output_frame').value
        self.height_filter = self.get_parameter('height_filter').value
        self.range_filter = self.get_parameter('range_filter').value
        self.min_height = self.get_parameter('min_height').value
        self.max_height = self.get_parameter('max_height').value
        self.min_range = self.get_parameter('min_range').value
        self.max_range = self.get_parameter('max_range').value
        self.odom_cancel = self.get_parameter('odom_cancel').value

    def radar_scan_callback(self, msg):
        self.sensor_frame = msg.header.frame_id
        marker_array = self.converter.convert_scan_to_markers(msg)
        if self.height_filter:
            marker_array = self.filter_by_height(marker_array)
        if self.range_filter:
            marker_array = self.filter_by_range(marker_array)
        if self.odom_cancel:
            try:
                tf_baselink2sensor = self.tf_buffer.lookup_transform('base_link', self.sensor_frame, msg.header.stamp)
                # cancel ego motion
                marker_array = self.ego_motion_canceller.cancel_ego_motion(msg.header.stamp ,marker_array, self.odom_queue, tf_baselink2sensor.transform)
            except Exception as e:
                self.get_logger().error(f"Failed to get transform: {e}")
                return

        self.marker_pub.publish(marker_array)

    def odom_callback(self, msg):
        self.odom_queue.append(msg)

    def filter_by_height(self, marker_array):
        new_marker_array = MarkerArray()
        for marker in marker_array.markers:
            z = marker.pose.position.z
            if z > self.max_height or z < self.min_height:
                continue
            new_marker_array.markers.append(marker)
        return new_marker_array

    def filter_by_range(self, marker_array):
        new_marker_array = MarkerArray()
        for marker in marker_array.markers:
            x = marker.pose.position.x
            y = marker.pose.position.y
            if math.sqrt(x**2 + y**2) < self.min_range or math.sqrt(x**2 + y**2) > self.max_range:
                continue
            new_marker_array.markers.append(marker)                
        return new_marker_array

def main(args=None):
    rclpy.init(args=args)
    node = RadarScanVisualizer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
