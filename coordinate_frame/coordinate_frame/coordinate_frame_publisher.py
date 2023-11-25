import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSReliabilityPolicy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Vector3, Pose, Quaternion
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import PoseStamped  # Import this

class CoordinateFramePublisher(Node):
    def __init__(self):
        qos = QoSProfile(depth=10,history=QoSHistoryPolicy.KEEP_LAST,reliability=QoSReliabilityPolicy.BEST_EFFORT)

        super().__init__('coordinate_frame_publisher')
        self.publisher = self.create_publisher(Marker, 'visualization_marker', 10)
        self.subscription = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.update_drone_pose, qos)
        self.drone_pose = Pose()
        self.timer = self.create_timer(1.0, self.publish_marker)

    def update_drone_pose(self, msg):
        self.drone_pose = msg.pose

    def create_marker(self, id, color, orientation):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.ns = "coordinate_frame"
        marker.id = id
        marker.type = marker.ARROW
        marker.action = marker.ADD
        marker.scale = Vector3(x=10, y=5, z=5)  # Adjust the scale accordingly
        marker.color = color

        marker.pose = self.drone_pose
        marker.pose.orientation.x = orientation[0]
        marker.pose.orientation.y = orientation[1]
        marker.pose.orientation.z = orientation[2]
        marker.pose.orientation.w = orientation[3]

        return marker

    def publish_marker(self):
        # Red arrow for X-axis (No rotation)
        marker_x = self.create_marker(0, ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0), (0.0, 0.0, 0.0, 1.0))
        self.publisher.publish(marker_x)

        # Green arrow for Y-axis (90 degrees rotation about Z-axis)
        marker_y = self.create_marker(1, ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0), (0.0, 0.0, 0.7071, 0.7071))  # Quaternion (x, y, z, w)
        self.publisher.publish(marker_y)

        # Blue arrow for Z-axis (90 degrees rotation about Y-axis)
        marker_z = self.create_marker(2, ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0), (0.0, -0.7071, 0.0, 0.7071))  # Quaternion (x, y, z, w)
        self.publisher.publish(marker_z)


def main(args=None):
    rclpy.init(args=args)
    node = CoordinateFramePublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
