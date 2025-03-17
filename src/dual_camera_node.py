import rclpy
from rclpy.node import Node
from launch_ros.actions import Node as LaunchNode
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer

class DualCameraNode(Node):
    def __init__(self):
        super().__init__('dual_camera_node')
        self.get_logger().info('Dual camera node has started')

    def generate_launch_description(self):
        return LaunchDescription([
            DeclareLaunchArgument('camera1_name', default_value='camera1'),
            DeclareLaunchArgument('camera2_name', default_value='camera2'),

            ComposableNodeContainer(
                name='camera_container',
                namespace='',
                package='rclcpp_components',
                executable='component_container_mt',
                composable_node_descriptions=[
                    ComposableNode(
                        package='camera_ros',
                        plugin='camera::CameraNode',
                        name=LaunchConfiguration('camera1_name'),
                        parameters=[
                            {'camera_id': 0},
                            {'frame_id': 'camera1_frame'}
                        ]
                    ),
                    ComposableNode(
                        package='camera_ros',
                        plugin='camera::CameraNode',
                        name=LaunchConfiguration('camera2_name'),
                        parameters=[
                            {'camera_id': 1},
                            {'frame_id': 'camera2_frame'}
                        ]
                    )
                ]
            )
        ])


def main(args=None):
    rclpy.init(args=args)
    node = DualCameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
