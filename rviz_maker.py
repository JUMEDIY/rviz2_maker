import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class CubeWithTextPublisher(Node):
    def __init__(self):
        super().__init__('cube_with_text_publisher')
        self.publisher_ = self.create_publisher(Marker, 'visualization_marker', 10)
        timer_period = 1.0  # วินาที
        self.timer = self.create_timer(timer_period, self.publish_markers)
    
    def publish_markers(self):
        # สร้าง cube marker
        cube_marker = Marker()
        cube_marker.header.frame_id = "map"
        cube_marker.header.stamp = self.get_clock().now().to_msg()
        cube_marker.ns = "basic_shapes"
        cube_marker.id = 0
        cube_marker.type = Marker.CUBE
        cube_marker.action = Marker.ADD

        # ตำแหน่งของ cube
        cube_marker.pose.position.x = 1.0
        cube_marker.pose.position.y = 1.0
        cube_marker.pose.position.z = 0.5

        # ขนาดของ cube
        cube_marker.scale.x = 1.0
        cube_marker.scale.y = 1.0
        cube_marker.scale.z = 1.0

        # สีของ cube (RGBA)
        cube_marker.color.r = 0.0
        cube_marker.color.g = 1.0
        cube_marker.color.b = 0.0
        cube_marker.color.a = 1.0

        # ระยะเวลาที่ cube จะปรากฏบน RViz2 (0 หมายถึงแสดงตลอด)
        cube_marker.lifetime = rclpy.duration.Duration(seconds=0).to_msg()

        # สร้าง text marker
        text_marker = Marker()
        text_marker.header.frame_id = "map"
        text_marker.header.stamp = self.get_clock().now().to_msg()
        text_marker.ns = "basic_shapes"
        text_marker.id = 1
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD

        # ตำแหน่งของ text (แสดงอยู่เหนือ cube)
        text_marker.pose.position.x = 1.0
        text_marker.pose.position.y = 1.0
        text_marker.pose.position.z = 1.5

        # ขนาดของ text
        text_marker.scale.z = 0.3  # ขนาดของตัวอักษร

        # สีของ text (RGBA)
        text_marker.color.r = 1.0
        text_marker.color.g = 0.0
        text_marker.color.b = 0.0
        text_marker.color.a = 1.0

        # ข้อความที่จะแสดง
        text_marker.text = "Cube Text Here"

        # ระยะเวลาที่ text จะปรากฏบน RViz2 (0 หมายถึงแสดงตลอด)
        text_marker.lifetime = rclpy.duration.Duration(seconds=0).to_msg()

        # Publish both markers
        self.publisher_.publish(cube_marker)
        self.publisher_.publish(text_marker)
        self.get_logger().info('Publishing Cube and Text Markers')

def main(args=None):
    rclpy.init(args=args)
    node = CubeWithTextPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
