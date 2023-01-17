# Copyright (c) 2022, NVIDIA CORPORATION. All rights reserved.
#
# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.

import os
import launch
from launch_ros.actions import ComposableNodeContainer, Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess


# detect all 36h11 tags
cfg_36h11 = {
    'image_transport': 'raw',
    'family': '36h11',
    'size': 0.162
}


def generate_launch_description():

    nav2_bringup_launch_dir = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch')

    use_sim_time = LaunchConfiguration('use_sim_time',
                                       default='True')

    nvblox_param_dir = LaunchConfiguration('nvblox_params_file',
                                           default=os.path.join(get_package_share_directory('nvblox_nav2'), 'params', 'nvblox.yaml'),)

    ############# ROS2 DECLARATIONS #############

    use_sim_dec = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Omniverse Isaac Sim) clock if true')

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(get_package_share_directory(
            'nvblox_nav2'), 'params', 'carter_nav2.yaml'),
        description='Full path to param file to load')

    use_depth_arg = DeclareLaunchArgument(
        'use_depth',
        default_value='True',
        description='Use depth as an input for nvblox reconstruction'
    )

    use_lidar_arg = DeclareLaunchArgument(
        'use_lidar',
        default_value='True',
        description='Use lidar as an input for nvblox reconstruction'
    )

    ############# ROS2 NODES #############

    apriltag_node = ComposableNode(
        name='apriltag',
        package='isaac_ros_apriltag',
        plugin='isaac_ros::apriltag::AprilTagNode',
        remappings=[('camera/image_rect', '/left/rgb'),
                    ('camera/camera_info', '/left/camera_info'),
                    ('tf', '/tf')],
        parameters=[cfg_36h11],
    )

    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='isaac_ros::visual_slam::VisualSlamNode',
        remappings=[('stereo_camera/left/camera_info', '/left/camera_info'),
                    ('stereo_camera/right/camera_info', '/right/camera_info'),
                    ('stereo_camera/left/image', '/left/rgb'),
                    ('stereo_camera/right/image', '/right/rgb'),
                    ('tf', '/tf')],
        parameters=[{
            'enable_rectified_pose': False,
            'denoise_input_images': False,
            'rectified_images': True,
            'enable_debug_mode': False,
            'enable_imu': False,
            'debug_dump_path': '/tmp/elbrus',
            'left_camera_frame': 'carter_camera_stereo_left',
            'right_camera_frame': 'carter_camera_stereo_right',
            'map_frame': 'map',
            'fixed_frame': 'odom',
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'current_smooth_frame': 'base_link_smooth',
            'current_rectified_frame': 'base_link_rectified',
            'enable_observations_view': True,
            'enable_landmarks_view': True,
            'enable_reading_slam_internals': True,
            'enable_slam_visualization': True,
            'enable_localization_n_mapping': True,
            # 'publish_odom_to_base_tf': False,
            # 'publish_map_to_odom_tf': False,
            'use_sim_time': use_sim_time
        }]
    )
    isaac_ros_launch_container = ComposableNodeContainer(
        name='isaac_ros_launch_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            visual_slam_node,
            apriltag_node
        ],
        output='screen'
    )

    nvblox_node = Node(
        package='nvblox_ros', executable='nvblox_node',
        parameters=[nvblox_param_dir,
                    {'use_sim_time': use_sim_time,
                     # 'global_frame': 'odom',
                     'use_depth': LaunchConfiguration('use_depth'),
                     'use_lidar': LaunchConfiguration('use_lidar')}],
        output='screen',
        remappings=[('depth/image', '/left/depth'),
                    ('depth/camera_info', '/left/camera_info'),
                    ('color/image', '/left/rgb'),
                    ('color/camera_info', '/left/camera_info'),
                    ('pointcloud', '/point_cloud')
                    ])

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_launch_dir, 'navigation_launch.py')),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time'),
                          'params_file': LaunchConfiguration('params_file'),
                          'autostart': 'True'}.items())

    # https://foxglove.dev/docs/studio/connection/ros2
    # https://github.com/foxglove/ros-foxglove-bridge
    foxglove_bridge_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        # output='screen'
    )

    cmd_wrapper_fix_node = Node(
        package='cmd_wrapper',
        executable='cmd_wrapper',
        output='screen'
    )

    ############################

    # Launch ROS2 packages
    ld = LaunchDescription()
    # Definitions
    ld.add_action(use_depth_arg)
    ld.add_action(params_file_arg)
    ld.add_action(use_lidar_arg)
    ld.add_action(use_sim_dec)
    # Foxglove
    ld.add_action(foxglove_bridge_node)
    # vSLAM and NVBLOX
    ld.add_action(visual_slam_launch_container)
    ld.add_action(nvblox_node)
    # Navigation tool
    # ld.add_action(nav2_launch)
    # Command sender TMP
    ld.add_action(cmd_wrapper_fix_node)

    return ld
# EOF
