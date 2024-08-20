import rclpy
from rclpy.node import Node
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker

class InteractiveCube(Node):
    def __init__(self):
        super().__init__('interactive_cube')
        self.server = InteractiveMarkerServer(self, "interactive_cube_marker")

        # สร้าง Interactive Marker
        interactive_marker = InteractiveMarker()
        interactive_marker.header.frame_id = "map"
        interactive_marker.name = "my_cube"
        interactive_marker.description = "Move the cube"
        interactive_marker.pose.position.x = 1.0
        interactive_marker.pose.position.y = 1.0
        interactive_marker.pose.position.z = 0.5

        # สร้าง Cube Marker
        cube_marker = Marker()
        cube_marker.type = Marker.CUBE
        cube_marker.scale.x = 1.0
        cube_marker.scale.y = 1.0
        cube_marker.scale.z = 1.0
        cube_marker.color.r = 0.0
        cube_marker.color.g = 1.0
        cube_marker.color.b = 0.0
        cube_marker.color.a = 1.0

        # สร้าง Interactive Marker Control
        control = InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append(cube_marker)
        interactive_marker.controls.append(control)

        # เพิ่มการควบคุมการเลื่อนในแกน X
        move_x_control = InteractiveMarkerControl()
        move_x_control.name = "move_x"
        move_x_control.orientation.w = 1.0  
        move_x_control.orientation.x = 1.0
        move_x_control.orientation.y = 0.0
        move_x_control.orientation.z = 0.0
        move_x_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        interactive_marker.controls.append(move_x_control)

        # เพิ่มการควบคุมการเลื่อนในแกน Y
        move_y_control = InteractiveMarkerControl()
        move_y_control.name = "move_y"
        move_y_control.orientation.w = 1.0  
        move_y_control.orientation.x = 0.0
        move_y_control.orientation.y = 1.0
        move_y_control.orientation.z = 0.0
        move_y_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        interactive_marker.controls.append(move_y_control)

        # เพิ่มการควบคุมการเลื่อนในแกน Z
        move_z_control = InteractiveMarkerControl()
        move_z_control.name = "move_z"
        move_z_control.orientation.w = 1.0  
        move_z_control.orientation.x = 0.0
        move_z_control.orientation.y = 0.0
        move_z_control.orientation.z = 1.0
        move_z_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        interactive_marker.controls.append(move_z_control)

        # เพิ่ม Interactive Marker ไปยังเซิร์ฟเวอร์
        self.server.insert(interactive_marker)  # ไม่ใส่ callback ที่นี่
        self.server.setCallback(interactive_marker.name, self.process_feedback)  # ใช้ setCallback เพื่อจัดการ feedback
        self.server.applyChanges()

    def process_feedback(self, feedback):
        self.get_logger().info(f"Marker moved to: {feedback.pose.position.x}, {feedback.pose.position.y}, {feedback.pose.position.z}")

def main(args=None):
    rclpy.init(args=args)
    node = InteractiveCube()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
