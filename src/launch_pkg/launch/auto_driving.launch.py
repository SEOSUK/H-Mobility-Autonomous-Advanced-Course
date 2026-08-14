import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    launch_pkg_share = get_package_share_directory('launch_pkg')
    workspace_root = os.path.dirname(
        os.path.dirname(
            os.path.dirname(
                os.path.dirname(launch_pkg_share)
            )
        )
    )
    default_model_path = os.path.join(
        workspace_root,
        'src',
        'skk_assign',
        'learned_data',
        'best.pt',
    )

    camera_device_arg = DeclareLaunchArgument(
        'camera_device',
        default_value='/dev/video3',
        description='Camera device path used by image_publisher_node.',
    )
    camera_index_arg = DeclareLaunchArgument(
        'camera_index',
        default_value='3',
        description='Fallback camera index used when camera_device is empty.',
    )
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='image_raw',
        description='Image topic published by the webcam node.',
    )
    detection_topic_arg = DeclareLaunchArgument(
        'detection_topic',
        default_value='detections',
        description='Detection topic published by the YOLO node.',
    )
    lane_topic_arg = DeclareLaunchArgument(
        'lane_topic',
        default_value='yolov8_lane_info',
        description='Lane info topic published by lane_info_extractor_node.',
    )
    roi_topic_arg = DeclareLaunchArgument(
        'roi_topic',
        default_value='roi_image',
        description='ROI image topic published by lane_info_extractor_node.',
    )
    edge_topic_arg = DeclareLaunchArgument(
        'edge_topic',
        default_value='lane2_edge_image',
        description='Binary lane image topic published by lane_info_extractor_node.',
    )
    path_topic_arg = DeclareLaunchArgument(
        'path_topic',
        default_value='path_planning_result',
        description='Path planning topic published by path_planner_node.',
    )
    control_topic_arg = DeclareLaunchArgument(
        'control_topic',
        default_value='topic_control_signal',
        description='Motion command topic consumed by serial_sender_node.',
    )
    control_input_topic_arg = DeclareLaunchArgument(
        'control_input_topic',
        default_value='auto_drive_command_raw',
        description='Raw motion command topic before Enter/c gating.',
    )
    save_interval_arg = DeclareLaunchArgument(
        'save_interval',
        default_value='1.0',
        description='Seconds between saved images.',
    )
    yolo_model_arg = DeclareLaunchArgument(
        'yolo_model',
        default_value=default_model_path,
        description='YOLO model path for lane segmentation.',
    )
    yolo_device_arg = DeclareLaunchArgument(
        'yolo_device',
        default_value='cpu',
        description='YOLO inference device such as cpu or cuda:0.',
    )
    yolo_threshold_arg = DeclareLaunchArgument(
        'yolo_threshold',
        default_value='0.5',
        description='YOLO confidence threshold.',
    )
    show_viewer_arg = DeclareLaunchArgument(
        'show_viewer',
        default_value='false',
        description='Open rqt_image_view for the camera topic.',
    )
    show_detection_viewer_arg = DeclareLaunchArgument(
        'show_detection_viewer',
        default_value='true',
        description='Open rqt_image_view for the YOLO overlay topic.',
    )
    show_binary_viewer_arg = DeclareLaunchArgument(
        'show_binary_viewer',
        default_value='true',
        description='Open one rqt_image_view popup for the binary lane image topic.',
    )
    show_image_arg = DeclareLaunchArgument(
        'show_image',
        default_value='false',
        description='Show the webcam image in an OpenCV window.',
    )
    show_lane_windows_arg = DeclareLaunchArgument(
        'show_lane_windows',
        default_value='true',
        description='Show lane edge, bird-eye, and ROI popup windows.',
    )
    enable_camera_arg = DeclareLaunchArgument(
        'enable_camera',
        default_value='true',
        description='Launch the image_publisher_node.',
    )
    enable_lane_detection_arg = DeclareLaunchArgument(
        'enable_lane_detection',
        default_value='true',
        description='Launch the YOLO lane detection pipeline.',
    )
    enable_path_planning_arg = DeclareLaunchArgument(
        'enable_path_planning',
        default_value='true',
        description='Launch the path_planner_node.',
    )
    enable_motion_planning_arg = DeclareLaunchArgument(
        'enable_motion_planning',
        default_value='true',
        description='Launch the motion_planner_node.',
    )
    enable_serial_sender_arg = DeclareLaunchArgument(
        'enable_serial_sender',
        default_value='true',
        description='Launch the serial_sender_node to send commands to the car.',
    )

    camera_device = LaunchConfiguration('camera_device')
    camera_index = LaunchConfiguration('camera_index')
    image_topic = LaunchConfiguration('image_topic')
    detection_topic = LaunchConfiguration('detection_topic')
    lane_topic = LaunchConfiguration('lane_topic')
    roi_topic = LaunchConfiguration('roi_topic')
    edge_topic = LaunchConfiguration('edge_topic')
    path_topic = LaunchConfiguration('path_topic')
    control_topic = LaunchConfiguration('control_topic')
    control_input_topic = LaunchConfiguration('control_input_topic')
    save_interval = LaunchConfiguration('save_interval')
    yolo_model = LaunchConfiguration('yolo_model')
    yolo_device = LaunchConfiguration('yolo_device')
    yolo_threshold = LaunchConfiguration('yolo_threshold')
    show_viewer = LaunchConfiguration('show_viewer')
    show_detection_viewer = LaunchConfiguration('show_detection_viewer')
    show_binary_viewer = LaunchConfiguration('show_binary_viewer')
    show_image = LaunchConfiguration('show_image')
    show_lane_windows = LaunchConfiguration('show_lane_windows')
    enable_camera = LaunchConfiguration('enable_camera')
    enable_lane_detection = LaunchConfiguration('enable_lane_detection')
    enable_path_planning = LaunchConfiguration('enable_path_planning')
    enable_motion_planning = LaunchConfiguration('enable_motion_planning')
    enable_serial_sender = LaunchConfiguration('enable_serial_sender')

    return LaunchDescription([
        camera_device_arg,
        camera_index_arg,
        image_topic_arg,
        detection_topic_arg,
        lane_topic_arg,
        roi_topic_arg,
        edge_topic_arg,
        path_topic_arg,
        control_topic_arg,
        control_input_topic_arg,
        save_interval_arg,
        yolo_model_arg,
        yolo_device_arg,
        yolo_threshold_arg,
        show_viewer_arg,
        show_detection_viewer_arg,
        show_binary_viewer_arg,
        show_image_arg,
        show_lane_windows_arg,
        enable_camera_arg,
        enable_lane_detection_arg,
        enable_path_planning_arg,
        enable_motion_planning_arg,
        enable_serial_sender_arg,
        Node(
            package='camera_perception_pkg',
            executable='image_publisher_node',
            name='auto_driving_image_publisher_node',
            output='log',
            condition=IfCondition(enable_camera),
            parameters=[{
                'data_source': 'camera',
                'cam_num': ParameterValue(camera_index, value_type=int),
                'camera_device': ParameterValue(camera_device, value_type=str),
                'pub_topic': ParameterValue(image_topic, value_type=str),
                'save_image': True,
                'save_interval': ParameterValue(save_interval, value_type=float),
                'logger': ParameterValue(show_image, value_type=bool),
            }],
        ),
        Node(
            package='camera_perception_pkg',
            executable='yolov8_node',
            name='auto_driving_yolov8_node',
            output='screen',
            condition=IfCondition(enable_lane_detection),
            parameters=[{
                'model': ParameterValue(yolo_model, value_type=str),
                'device': ParameterValue(yolo_device, value_type=str),
                'threshold': ParameterValue(yolo_threshold, value_type=float),
                'print_green_box_size': True,
            }],
            arguments=['--ros-args', '--log-level', 'fatal'],
            remappings=[
                ('image_raw', image_topic),
                ('detections', detection_topic),
            ],
        ),
        Node(
            package='camera_perception_pkg',
            executable='lane_info_extractor_node',
            name='auto_driving_lane_info_extractor_node',
            output='log',
            condition=IfCondition(enable_lane_detection),
            parameters=[{
                'sub_detection_topic': ParameterValue(detection_topic, value_type=str),
                'pub_topic': ParameterValue(lane_topic, value_type=str),
                'show_image': ParameterValue(show_lane_windows, value_type=bool),
            }],
            remappings=[
                ('lane2_edge_image', edge_topic),
                ('roi_image', roi_topic),
            ],
        ),
        Node(
            package='camera_perception_pkg',
            executable='traffic_light_detector_node',
            name='auto_driving_traffic_light_detector_node',
            output='log',
            condition=IfCondition(enable_lane_detection),
            parameters=[{
                'sub_detection_topic': ParameterValue(detection_topic, value_type=str),
                'sub_image_topic': ParameterValue(image_topic, value_type=str),
            }],
            remappings=[
                ('detections', detection_topic),
                ('image_raw', image_topic),
            ],
        ),
        Node(
            package='decision_making_pkg',
            executable='path_planner_node',
            name='auto_driving_path_planner_node',
            output='log',
            condition=IfCondition(enable_path_planning),
            parameters=[{
                'sub_lane_topic': ParameterValue(lane_topic, value_type=str),
                'pub_topic': ParameterValue(path_topic, value_type=str),
            }],
        ),
        Node(
            package='decision_making_pkg',
            executable='motion_planner_node',
            name='auto_driving_motion_planner_node',
            output='screen',
            condition=IfCondition(enable_motion_planning),
            parameters=[{
                'sub_detection_topic': ParameterValue(detection_topic, value_type=str),
                'sub_lane_topic': ParameterValue(path_topic, value_type=str),
                'pub_topic': ParameterValue(control_input_topic, value_type=str),
            }],
        ),
        Node(
            package='launch_pkg',
            executable='auto_drive_control.py',
            name='auto_drive_control_node',
            output='log',
            condition=IfCondition(enable_motion_planning),
            parameters=[{
                'sub_topic': ParameterValue(control_input_topic, value_type=str),
                'pub_topic': ParameterValue(control_topic, value_type=str),
            }],
        ),
        Node(
            package='serial_communication_pkg',
            executable='serial_sender_node',
            name='auto_driving_serial_sender_node',
            output='log',
            condition=IfCondition(enable_serial_sender),
            parameters=[{
                'sub_topic': ParameterValue(control_topic, value_type=str),
            }],
        ),
        Node(
            package='debug_pkg',
            executable='yolov8_visualizer_node',
            name='auto_driving_yolov8_visualizer_node',
            output='log',
            condition=IfCondition(enable_lane_detection),
            remappings=[
                ('image_raw', image_topic),
                ('detections', detection_topic),
                ('yolov8_visualized_img', 'auto_driving_yolov8_visualized_img'),
            ],
        ),
        ExecuteProcess(
            condition=IfCondition(show_viewer),
            cmd=['ros2', 'run', 'rqt_image_view', 'rqt_image_view', image_topic],
            output='log',
        ),
        ExecuteProcess(
            condition=IfCondition(show_detection_viewer),
            cmd=['ros2', 'run', 'rqt_image_view', 'rqt_image_view', 'auto_driving_yolov8_visualized_img'],
            output='log',
        ),
        ExecuteProcess(
            condition=IfCondition(show_binary_viewer),
            cmd=['ros2', 'run', 'rqt_image_view', 'rqt_image_view', edge_topic],
            output='log',
        ),
    ])
