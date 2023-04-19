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


LOCAL_PATH = "/workspaces/isaac_ros-dev"


def generate_launch_description():

    pkg_description = get_package_share_directory('carter_description')
    bringup_dir = get_package_share_directory('nvblox_examples_bringup')
    pkg_isaac_demo = get_package_share_directory('isaac_demo')

    use_sim_time = LaunchConfiguration('use_sim_time',
                                       default='True')

    # Bi3DNode parameters
    featnet_engine_file_path = LaunchConfiguration('featnet_engine_file_path')
    segnet_engine_file_path = LaunchConfiguration('segnet_engine_file_path')
    max_disparity_values = LaunchConfiguration('max_disparity_values')

    # FreespaceSegmentationNode parameters
    f_x_ = LaunchConfiguration('f_x')
    f_y_ = LaunchConfiguration('f_y')
    grid_height = LaunchConfiguration('grid_height')
    grid_width = LaunchConfiguration('grid_width')
    grid_resolution = LaunchConfiguration('grid_resolution')

    ############# ROS2 DECLARATIONS #############

    # Launch Arguments
    run_rviz_arg = DeclareLaunchArgument(
        'run_rviz', default_value='True',
        description='Whether to start RVIZ')

    use_sim_dec = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Omniverse Isaac Sim) clock if true')

    featnet_engine_file_arg = DeclareLaunchArgument(
        'featnet_engine_file_path',
        default_value=f"{LOCAL_PATH}/bi3d/bi3dnet_featnet.plan",
        description='The absolute path to the Bi3D Featnet TensorRT engine plan')
    segnet_engine_file_arg = DeclareLaunchArgument(
        'segnet_engine_file_path',
        default_value=f"{LOCAL_PATH}/bi3d/bi3dnet_segnet.plan",
        description='The absolute path to the Bi3D Segnet TensorRT engine plan')

    max_disparity_values_arg = DeclareLaunchArgument(
        'max_disparity_values',
        default_value='10',
        description='The maximum number of disparity values given for Bi3D inference')
    f_x_arg = DeclareLaunchArgument(
        'f_x',
        default_value='1465.99853515625',
        description='The number of pixels per distance unit in the x dimension')
    f_y_arg = DeclareLaunchArgument(
        'f_y',
        default_value='1468.2335205078125',
        description='The number of pixels per distance unit in the y dimension')
    grid_height_arg = DeclareLaunchArgument(
        'grid_height',
        default_value='2000',
        description='The desired height of the occupancy grid, in cells')
    grid_width_arg = DeclareLaunchArgument(
        'grid_width',
        default_value='2000',
        description='The desired width of the occupancy grid, in cells')
    grid_resolution_arg = DeclareLaunchArgument(
        'grid_resolution',
        default_value='0.01',
        description='The desired resolution of the occupancy grid, in m/cell')

    ############# ROS2 NODES #############
    ############# ISAAC ROS NODES ########

    apriltag_node = ComposableNode(
        name='apriltag',
        package='isaac_ros_apriltag',
        plugin='nvidia::isaac_ros::apriltag::AprilTagNode',
        remappings=[('/image', '/front/stereo_camera/left/rgb'),
                    ('/camera_info', '/front/stereo_camera/left/camera_info'),
                    ],
        parameters=[{'size': 0.32,
                     'max_tags': 64}],
    )

    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='isaac_ros::visual_slam::VisualSlamNode',
        remappings=[('stereo_camera/left/camera_info', '/front/stereo_camera/left/camera_info'),
                    ('stereo_camera/right/camera_info', '/front/stereo_camera/right/camera_info'),
                    ('stereo_camera/left/image', '/front/stereo_camera/left/rgb'),
                    ('stereo_camera/right/image', '/front/stereo_camera/right/rgb'),
                    ('visual_slam/tracking/odometry', '/odom'),
                    ],
        parameters=[{
            # 'enable_rectified_pose': False,
            'denoise_input_images': True,
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
            'publish_map_to_odom_tf': False,
            'use_sim_time': use_sim_time
        }]
    )

    bi3d_node = ComposableNode(
        name='bi3d_node',
        package='isaac_ros_bi3d',
        plugin='nvidia::isaac_ros::bi3d::Bi3DNode',
        parameters=[{
                'featnet_engine_file_path': featnet_engine_file_path,
                'segnet_engine_file_path': segnet_engine_file_path,
                'max_disparity_values': max_disparity_values,
                'disparity_values': [3, 6, 9, 12, 15, 18, 21, 24, 27, 30],
                'use_sim_time': use_sim_time}],
        remappings=[('bi3d_node/bi3d_output', 'bi3d_mask'),
                    ('left_image_bi3d', '/front/stereo_camera/left/rgb'),
                    ('right_image_bi3d', '/front/stereo_camera/right/rgb')]
    )

    freespace_segmentation_node = ComposableNode(
        name='freespace_segmentation_node',
        package='isaac_ros_bi3d_freespace',
        plugin='nvidia::isaac_ros::bi3d_freespace::FreespaceSegmentationNode',
        parameters=[{
            'base_link_frame': 'base_link',
            'camera_frame': 'carter_camera_stereo_left',
            'f_x': f_x_,
            'f_y': f_y_,
            'grid_height': grid_height,
            'grid_width': grid_width,
            'grid_resolution': grid_resolution,
            'use_sim_time': use_sim_time
        }])

    isaac_ros_launch_container = ComposableNodeContainer(
        name='isaac_ros_launch_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            visual_slam_node,
            apriltag_node,
            bi3d_node,
            freespace_segmentation_node,
        ],
        output='screen'
    )

    ############# OTHER ROS2 NODES #######

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            bringup_dir, 'launch', 'nav2', 'nav2_isaac_sim.launch.py')))

    # Nvblox
    nvblox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(bringup_dir, 'launch', 'nvblox', 'nvblox.launch.py')]),
        launch_arguments={'setup_for_isaac_sim': 'True'}.items())

    # https://foxglove.dev/docs/studio/connection/ros2
    # https://github.com/foxglove/ros-foxglove-bridge
    foxglove_bridge_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        # output='screen'
    )

    # include another launch file in nanosaur namespace
    # https://docs.ros.org/en/foxy/How-To-Guides/Launch-file-different-formats.html
    description_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([pkg_description, '/launch/description.launch.py']),
            )

    ############################

    # Launch ROS2 packages
    ld = LaunchDescription()
    # Definitions
    ld.add_action(use_sim_dec)
    ld.add_action(featnet_engine_file_arg)
    ld.add_action(segnet_engine_file_arg)
    ld.add_action(max_disparity_values_arg)
    ld.add_action(f_x_arg)
    ld.add_action(f_y_arg)
    ld.add_action(grid_height_arg)
    ld.add_action(grid_width_arg)
    ld.add_action(grid_resolution_arg)
    # carter description launch
    ld.add_action(description_launch)
    # Foxglove
    #ld.add_action(foxglove_bridge_node)
    # Isaac ROS container
    ld.add_action(isaac_ros_launch_container)
    # vSLAM and NVBLOX
    ld.add_action(nvblox_launch)
    # Navigation tool
    ld.add_action(nav2_launch)

    return ld
# EOF
