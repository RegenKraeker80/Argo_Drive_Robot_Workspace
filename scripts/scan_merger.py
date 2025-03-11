#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import numpy as np

class LaserScanMerger(Node):
    def __init__(self):
        super().__init__('laser_scan_merger')

        self.use_sim_time = self.get_parameter_or('use_sim_time', False)

        self.declare_parameter('output_topic', '/merged_scan')
        self.declare_parameter('scan1_topic', '/scan1')
        self.declare_parameter('scan2_topic', '/scan2')
        self.declare_parameter('scan1_frame', 'scan1')
        self.declare_parameter('scan2_frame', 'scan2')
        self.declare_parameter('target_frame', 'scan_middle_link')

        self.output_topic = self.get_parameter('output_topic').value
        self.scan1_topic = self.get_parameter('scan1_topic').value
        self.scan2_topic = self.get_parameter('scan2_topic').value
        self.scan1_frame = self.get_parameter('scan1_frame').value
        self.scan2_frame = self.get_parameter('scan2_frame').value
        self.target_frame = self.get_parameter('target_frame').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.scan1 = None
        self.scan2 = None

        self.scan1_sub = self.create_subscription(LaserScan, self.scan1_topic, self.scan1_callback, 10)
        self.scan2_sub = self.create_subscription(LaserScan, self.scan2_topic, self.scan2_callback, 10)
        self.publisher = self.create_publisher(LaserScan, self.output_topic, 10)


    def scan1_callback(self, msg):
        transformed_scan = self.transform_scan(msg, self.scan1_frame)
        if transformed_scan:
            self.scan1 = transformed_scan
            self.merge_scans()

    def scan2_callback(self, msg):
        transformed_scan = self.transform_scan(msg, self.scan2_frame)
        if transformed_scan:
            self.scan2 = transformed_scan
            self.merge_scans()

    def transform_scan(self, scan_msg, source_frame):
        """Transformiert einen LaserScan ins kartesische System und dann ins Ziel-Frame"""
        try:
            trans = self.tf_buffer.lookup_transform(self.target_frame, source_frame, rclpy.time.Time(), rclpy.duration.Duration(seconds=1.0))

            tx, ty = trans.transform.translation.x, trans.transform.translation.y
            qz, qw = trans.transform.rotation.z, trans.transform.rotation.w
            theta = 2.0 * np.arctan2(qz, qw)

            angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(scan_msg.ranges))
            ranges = np.array(scan_msg.ranges)

            # Ungültige Werte herausfiltern
            valid_indices = np.where((ranges >= scan_msg.range_min) & (ranges <= scan_msg.range_max))
            ranges, angles = ranges[valid_indices], angles[valid_indices]

            # Kartesische Koordinaten (X, Y)
            x = ranges * np.cos(angles)
            y = ranges * np.sin(angles)

            # Transformation ins Ziel-Frame
            x_transformed = x * np.cos(theta) - y * np.sin(theta) + tx
            y_transformed = x * np.sin(theta) + y * np.cos(theta) + ty


            # Zurück zu Polarkoordinaten
            transformed_ranges = np.hypot(x_transformed, y_transformed)
            transformed_angles = np.arctan2(y_transformed, x_transformed)

            transformed_scan = LaserScan()
            transformed_scan.header.stamp = scan_msg.header.stamp
            transformed_scan.header.frame_id = self.target_frame
            transformed_scan.angle_min = -np.pi
            transformed_scan.angle_max = np.pi
            transformed_scan.angle_increment = scan_msg.angle_increment
            transformed_scan.range_min = scan_msg.range_min
            transformed_scan.range_max = scan_msg.range_max

            # Initialisiere Scan-Daten mit max-Werten
            num_points = int((transformed_scan.angle_max - transformed_scan.angle_min) / transformed_scan.angle_increment)
            merged_ranges = [float('inf')] * num_points

            indices = np.round((transformed_angles - transformed_scan.angle_min) / transformed_scan.angle_increment, 6)
            indices = indices.astype(int)
            for i, index in enumerate(indices):
                if 0 <= index < num_points:
                    merged_ranges[index] = min(merged_ranges[index], transformed_ranges[i])

            transformed_scan.ranges = merged_ranges
            return transformed_scan

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"TF Transform fehlgeschlagen: {str(e)}")
            return None

    def merge_scans(self):

        """Führt beide transformierten Scans zu einem 360°-Scan zusammen"""

        if self.scan1 is None or self.scan2 is None:
            self.get_logger().warn("Scans noch nicht verfügbar!")
            return

        angle_min = -np.pi
        angle_max = np.pi
        angle_increment = max(self.scan1.angle_increment, self.scan2.angle_increment)
        num_points = int(round((angle_max - angle_min) / angle_increment)) + 1
        merged_ranges = [float('inf')] * num_points

        def insert_scan(scan):
            angles = np.linspace(scan.angle_min, scan.angle_max, len(scan.ranges),6)
            indices = np.round((angles - angle_min) / angle_increment)
            indices = indices.astype(int)
            for i, index in enumerate(indices):
                if 0 <= index < num_points:
                    value = scan.ranges[i]
                    merged_ranges[index] = min(merged_ranges[index], value)

        insert_scan(self.scan1)
        insert_scan(self.scan2)

        merged_scan = LaserScan()
        merged_scan.header.stamp = self.get_clock().now().to_msg() if self.use_sim_time else rclpy.time.Time().to_msg()
        merged_scan.header.frame_id = self.target_frame
        merged_scan.angle_min = angle_min
        merged_scan.angle_max = angle_max
        merged_scan.angle_increment = angle_increment
        merged_scan.range_min = min(self.scan1.range_min, self.scan2.range_min)
        merged_scan.range_max = max(self.scan1.range_max, self.scan2.range_max)
        merged_scan.ranges = merged_ranges

        self.publisher.publish(merged_scan)

def main():
    rclpy.init()
    node = LaserScanMerger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
