import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs_py.point_cloud2 import read_points  # Corrected import for point_cloud2
import numpy as np
import open3d as o3d

class PointCloudProcessor(Node):
    def __init__(self):
        super().__init__('pointcloud_processor')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/pointcloud_topic',  # Change this to your topic name
            self.pointcloud_callback,
            10
        )
        self.publisher = self.create_publisher(MarkerArray, '/bounding_boxes', 10)
        self.get_logger().info('PointCloud Processor Node Started.')

    def pointcloud_callback(self, msg):
        # Convert ROS2 PointCloud2 to numpy array
        points = self.pointcloud2_to_xyz(msg)
        if points is None:
            return

        # Preprocess point cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        
        # Apply voxel downsampling
        pcd = pcd.voxel_down_sample(voxel_size=0.05)

        # Segment ground plane (RANSAC)
        plane_model, inliers = pcd.segment_plane(distance_threshold=0.02,
                                                 ransac_n=3,
                                                 num_iterations=1000)
        pcd_objects = pcd.select_by_index(inliers, invert=True)

        # Cluster objects
        labels = np.array(pcd_objects.cluster_dbscan(eps=0.2, min_points=10))
        unique_labels = np.unique(labels)

        # Generate bounding boxes
        markers = MarkerArray()
        for i, label in enumerate(unique_labels):
            if label == -1:  # Ignore noise points
                continue
            cluster = pcd_objects.select_by_index(np.where(labels == label)[0])
            bbox = cluster.get_axis_aligned_bounding_box()
            markers.markers.append(self.create_bbox_marker(bbox, i))

        # Publish bounding boxes
        self.publisher.publish(markers)

    def pointcloud2_to_xyz(self, cloud_msg):
        """ Convert ROS2 PointCloud2 message to numpy array """
        points_list = []
        for point in read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True):
            points_list.append([point[0], point[1], point[2]])
        return np.array(points_list) if points_list else None

    def create_bbox_marker(self, bbox, marker_id):
        """ Create a bounding box marker """
        marker = Marker()
        marker.header.frame_id = "map"  # Change to match your frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "bboxes"
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = (bbox.min_bound[0] + bbox.max_bound[0]) / 2
        marker.pose.position.y = (bbox.min_bound[1] + bbox.max_bound[1]) / 2
        marker.pose.position.z = (bbox.min_bound[2] + bbox.max_bound[2]) / 2
        marker.scale.x = bbox.max_bound[0] - bbox.min_bound[0]
        marker.scale.y = bbox.max_bound[1] - bbox.min_bound[1]
        marker.scale.z = bbox.max_bound[2] - bbox.min_bound[2]
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.5
        return marker

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

