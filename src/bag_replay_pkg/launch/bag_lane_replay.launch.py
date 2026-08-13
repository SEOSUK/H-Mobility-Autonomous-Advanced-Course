from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_model_path = PathJoinSubstitution(
        [FindPackageShare('bag_replay_pkg'), 'models', 'best.pt']
    )

    model_arg = DeclareLaunchArgument(
        'model',
        default_value=default_model_path,
        description='YOLO model used during bag replay.',
    )
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='image_raw',
        description='Recorded image topic to subscribe to.',
    )
    device_arg = DeclareLaunchArgument(
        'device',
        default_value='cpu',
        description='YOLO inference device, for example cpu or cuda:0.',
    )
    threshold_arg = DeclareLaunchArgument(
        'threshold',
        default_value='0.5',
        description='YOLO confidence threshold.',
    )
    show_lane_windows_arg = DeclareLaunchArgument(
        'show_lane_windows',
        default_value='true',
        description='Show OpenCV windows from lane_info_extractor_node.',
    )

    image_topic = LaunchConfiguration('image_topic')
    model = LaunchConfiguration('model')
    device = LaunchConfiguration('device')
    threshold = LaunchConfiguration('threshold')
    show_lane_windows = LaunchConfiguration('show_lane_windows')

    bag_detection_topic = 'bag_detections'
    bag_lane_topic = 'bag_yolov8_lane_info'
    bag_roi_topic = 'bag_roi_image'
    bag_path_topic = 'bag_path_planning_result'
    bag_yolo_overlay_topic = 'bag_yolov8_visualized_img'
    bag_path_overlay_topic = 'bag_path_visualized_img'

    return LaunchDescription([
        model_arg,
        image_topic_arg,
        device_arg,
        threshold_arg,
        show_lane_windows_arg,
        Node(
            package='camera_perception_pkg',
            executable='yolov8_node',
            name='bag_yolov8_node',
            output='screen',
            parameters=[{
                'model': model,
                'device': device,
                'threshold': threshold,
            }],
            remappings=[
                ('image_raw', image_topic),
                ('detections', bag_detection_topic),
            ],
        ),
        Node(
            package='camera_perception_pkg',
            executable='lane_info_extractor_node',
            name='bag_lane_info_extractor_node',
            output='screen',
            parameters=[{
                'sub_detection_topic': bag_detection_topic,
                'pub_topic': bag_lane_topic,
                'show_image': show_lane_windows,
            }],
            remappings=[
                ('roi_image', bag_roi_topic),
            ],
        ),
        Node(
            package='decision_making_pkg',
            executable='path_planner_node',
            name='bag_path_planner_node',
            output='screen',
            parameters=[{
                'sub_lane_topic': bag_lane_topic,
                'pub_topic': bag_path_topic,
            }],
        ),
        Node(
            package='debug_pkg',
            executable='yolov8_visualizer_node',
            name='bag_yolov8_visualizer_node',
            output='screen',
            remappings=[
                ('image_raw', image_topic),
                ('detections', bag_detection_topic),
                ('yolov8_visualized_img', bag_yolo_overlay_topic),
            ],
        ),
        Node(
            package='debug_pkg',
            executable='path_visualizer_node',
            name='bag_path_visualizer_node',
            output='screen',
            parameters=[{
                'sub_roi_image_topic': bag_roi_topic,
                'sub_spline_path_topic': bag_path_topic,
                'pub_topic': bag_path_overlay_topic,
            }],
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'rqt_image_view', 'rqt_image_view', image_topic],
            output='screen',
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'rqt_image_view', 'rqt_image_view', bag_yolo_overlay_topic],
            output='screen',
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'rqt_image_view', 'rqt_image_view', bag_path_overlay_topic],
            output='screen',
        ),
    ])
