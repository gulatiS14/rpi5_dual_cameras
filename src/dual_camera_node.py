import rclpy
from rclpy.node import Node
from launch_ros.actions import Node as LaunchNode
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer

class DualCameraLauncher(Node):
    def __init__(self):
        super().__init__('dual_camera_launcher')

    def generate_launch_description(self):
        return LaunchDescription([
            # Declare camera names
            DeclareLaunchArgument('camera1_name', default_value='camera1'),
            DeclareLaunchArgument('camera2_name', default_value='camera2'),
            
            # Composable container for both cameras
            ComposableNodeContainer(
                name='camera_container',
                namespace='',
                package='rclcpp_components',
                executable='component_container_mt',
                composable_node_descriptions=[
                    # Camera 1
                    ComposableNode(
                        package='camera_ros',
                        plugin='camera::CameraNode',
                        name=LaunchConfiguration('camera1_name'),
                        parameters=[
                            {'camera': 0},  # Use index 0
                            {'frame_id': 'camera1_frame'}
                        ],
                        extra_arguments=[{'use_intra_process_comms': True}]
                    ),
                    # Camera 2
                    ComposableNode(
                        package='camera_ros',
                        plugin='camera::CameraNode',
                        name=LaunchConfiguration('camera2_name'),
                        parameters=[
                            {'camera': 1},  # Use index 1
                            {'frame_id': 'camera2_frame'}
                        ],
                        extra_arguments=[{'use_intra_process_comms': True}]
                    )
                ],
                output='screen'
            )
        ])

def main(args=None):
    rclpy.init(args=args)
    node = DualCameraLauncher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()