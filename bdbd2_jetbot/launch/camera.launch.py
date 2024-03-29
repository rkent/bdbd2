#
# Launch NVIDIA JetBot motor controller and camera nodes.
# This is for the original NVIDIA JetBot.
#
import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    video_source = Node(package='ros_deep_learning', executable='video_source',
                        parameters=[
                            {"resource": "csi://0"},
                            {"width": 320},
                            {"height": 240},
                            {"framerate": 15.0},
                            {"flip": "none"},
                            # https://github.com/dusty-nv/ros_deep_learning/pull/95/commits/abd91a3e4fb9b0aac5c6ed6ffd61355ef5277971
                            {"input_latency": 100},
                            {"frame_id": "pantilt_camera"},
                        ],
                        remappings=[
                            ("/video_source/raw", "/jetbot/camera/image_raw"),
                        ],
                        output='screen', emulate_tty=True)
    
    rtp_output = DeclareLaunchArgument('rtp_output', default_value="DUSTINF-LT1.fios-router.home:1234")
    
    video_output = Node(package='ros_deep_learning', executable='video_output',
                        parameters=[
                            {"resource": ["rtp://", LaunchConfiguration('rtp_output')]},
                            {"codec": "h264"},
                        ],
                        remappings=[
                            ("/video_output/image_in", "/jetbot/camera/image_raw"),
                        ],
                        output='screen', emulate_tty=True)
                        
    return LaunchDescription([
        rtp_output,
        video_source,
        video_output
    ])