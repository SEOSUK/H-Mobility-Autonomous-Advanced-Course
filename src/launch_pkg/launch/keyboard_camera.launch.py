import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    launch_pkg_share = get_package_share_directory('launch_pkg')
    workspace_root = os.path.dirname(
        os.path.dirname(
            os.path.dirname(
                os.path.dirname(launch_pkg_share)
            )
        )
    )
    keyboard_script = PathJoinSubstitution(
        [FindPackageShare('launch_pkg'), 'data_collection', 'data_collection.py']
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
        description='Lane edge image topic published by lane_info_extractor_node.',
    )
    bird_topic_arg = DeclareLaunchArgument(
        'bird_topic',
        default_value='lane2_bird_image',
        description='Bird-eye image topic published by lane_info_extractor_node.',
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
        description='Launch the image_publisher_node together with keyboard control.',
    )
    enable_lane_detection_arg = DeclareLaunchArgument(
        'enable_lane_detection',
        default_value='true',
        description='Launch the YOLO lane detection pipeline.',
    )
    keyboard_use_sudo_arg = DeclareLaunchArgument(
        'keyboard_use_sudo',
        default_value='true',
        description='Run data_collection.py with sudo because the keyboard library requires root on Linux.',
    )

    camera_device = LaunchConfiguration('camera_device')
    camera_index = LaunchConfiguration('camera_index')
    image_topic = LaunchConfiguration('image_topic')
    detection_topic = LaunchConfiguration('detection_topic')
    lane_topic = LaunchConfiguration('lane_topic')
    roi_topic = LaunchConfiguration('roi_topic')
    edge_topic = LaunchConfiguration('edge_topic')
    bird_topic = LaunchConfiguration('bird_topic')
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
    keyboard_use_sudo = LaunchConfiguration('keyboard_use_sudo')

    return LaunchDescription([
        camera_device_arg,
        camera_index_arg,
        image_topic_arg,
        detection_topic_arg,
        lane_topic_arg,
        roi_topic_arg,
        edge_topic_arg,
        bird_topic_arg,
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
        keyboard_use_sudo_arg,
        ExecuteProcess(
            condition=IfCondition(keyboard_use_sudo),
            cmd=['sudo', '-E', 'python3', keyboard_script],
            output='screen',
            emulate_tty=True,
            additional_env={'SKK_DISABLE_DATA_COLLECTION_CAMERA': '1'},
        ),
        ExecuteProcess(
            condition=UnlessCondition(keyboard_use_sudo),
            cmd=['python3', keyboard_script],
            output='screen',
            emulate_tty=True,
            additional_env={'SKK_DISABLE_DATA_COLLECTION_CAMERA': '1'},
        ),
        Node(
            package='camera_perception_pkg',
            executable='image_publisher_node',
            name='keyboard_camera_image_publisher_node',
            output='screen',
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
            name='keyboard_yolov8_node',
            output='screen',
            condition=IfCondition(enable_lane_detection),
            parameters=[{
                'model': ParameterValue(yolo_model, value_type=str),
                'device': ParameterValue(yolo_device, value_type=str),
                'threshold': ParameterValue(yolo_threshold, value_type=float),
            }],
            remappings=[
                ('image_raw', image_topic),
                ('detections', detection_topic),
            ],
        ),
        Node(
            package='camera_perception_pkg',
            executable='lane_info_extractor_node',
            name='keyboard_lane_info_extractor_node',
            output='screen',
            condition=IfCondition(enable_lane_detection),
            parameters=[{
                'sub_detection_topic': ParameterValue(detection_topic, value_type=str),
                'pub_topic': ParameterValue(lane_topic, value_type=str),
                'show_image': ParameterValue(show_lane_windows, value_type=bool),
            }],
            remappings=[
                ('lane2_edge_image', edge_topic),
                ('lane2_bird_image', bird_topic),
                ('roi_image', roi_topic),
            ],
        ),
        Node(
            package='debug_pkg',
            executable='yolov8_visualizer_node',
            name='keyboard_yolov8_visualizer_node',
            output='screen',
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
            output='screen',
        ),
        ExecuteProcess(
            condition=IfCondition(show_detection_viewer),
            cmd=['ros2', 'run', 'rqt_image_view', 'rqt_image_view', 'auto_driving_yolov8_visualized_img'],
            output='screen',
        ),
        ExecuteProcess(
            condition=IfCondition(show_binary_viewer),
            cmd=['ros2', 'run', 'rqt_image_view', 'rqt_image_view', edge_topic],
            output='screen',
        ),
    ])
