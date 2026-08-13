from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_perception_pkg',
            executable='lane_info_extractor_node',
            name='pc_lane_info_extractor_node',
            output='screen',
            parameters=[{
                'sub_detection_topic': 'detections',
                'pub_topic': 'pc_yolov8_lane_info',
                'show_image': True,
            }],
            remappings=[
                ('roi_image', 'pc_roi_image'),
            ],
        ),
        Node(
            package='debug_pkg',
            executable='yolov8_visualizer_node',
            name='pc_yolov8_visualizer_node',
            output='screen',
            remappings=[
                ('yolov8_visualized_img', 'pc_yolov8_visualized_img'),
            ],
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'rqt_image_view', 'rqt_image_view', '/image_raw'],
            output='screen',
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'rqt_image_view', 'rqt_image_view', '/pc_yolov8_visualized_img'],
            output='screen',
        ),
    ])
