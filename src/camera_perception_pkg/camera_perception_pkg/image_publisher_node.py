import rclpy 
from rclpy.node import Node 
from sensor_msgs.msg import Image 
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError

from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy

import sys
import cv2
import os
from datetime import datetime

#---------------Variable Setting---------------
# Publish할 토픽 이름
PUB_TOPIC_NAME = 'image_raw'

# 데이터 입력 소스: 'camera', 'image', 또는 'video' 중 택1하여 입력
DATA_SOURCE = 'video'

# 카메라(웹캠) 장치 번호 (ls /dev/video* 명령을 터미널 창에 입력하여 확인)
CAM_NUM = 0

# 카메라 장치 경로를 직접 지정하고 싶으면 사용 (예: /dev/video2)
CAMERA_DEVICE = ''

# 이미지 데이터가 들어있는 디렉토리의 경로를 입력
IMAGE_DIRECTORY_PATH = 'src/camera_perception_pkg/camera_perception_pkg/lib/Collected_Datasets/sample_dataset'

# 비디오 데이터 파일의 경로를 입력
VIDEO_FILE_PATH = 'src/camera_perception_pkg/camera_perception_pkg/lib/Collected_Datasets/driving_simulation.mp4'

# 화면에 publish하는 이미지를 띄울것인지 여부: True, 또는 False 중 택1하여 입력
SHOW_IMAGE = True

# 이미지 발행 주기 (초) - 소수점 필요 (int형은 반영되지 않음)
TIMER = 0.03

SAVE_IMAGE = False
SAVE_INTERVAL = 1.0
#----------------------------------------------

class ImagePublisherNode(Node):
    def __init__(self, data_source=DATA_SOURCE, cam_num=CAM_NUM, camera_device=CAMERA_DEVICE, img_dir=IMAGE_DIRECTORY_PATH, video_path=VIDEO_FILE_PATH, pub_topic=PUB_TOPIC_NAME, logger=SHOW_IMAGE, timer=TIMER, save_image=SAVE_IMAGE, save_interval=SAVE_INTERVAL):
        super().__init__('image_publisher_node')
        self.declare_parameter('data_source', data_source)
        self.declare_parameter('cam_num', cam_num)
        self.declare_parameter('camera_device', camera_device)
        self.declare_parameter('img_dir', img_dir)
        self.declare_parameter('video_path', video_path)
        self.declare_parameter('pub_topic', pub_topic)
        self.declare_parameter('logger', logger)
        self.declare_parameter('timer', timer)
        self.declare_parameter('save_image', save_image)
        self.declare_parameter('save_interval', save_interval)
        
        self.data_source = self.get_parameter('data_source').get_parameter_value().string_value
        self.cam_num = self.get_parameter('cam_num').get_parameter_value().integer_value
        self.camera_device = self.get_parameter('camera_device').get_parameter_value().string_value
        self.img_dir = self.get_parameter('img_dir').get_parameter_value().string_value
        self.video_path = self.get_parameter('video_path').get_parameter_value().string_value
        self.pub_topic = self.get_parameter('pub_topic').get_parameter_value().string_value
        self.logger = self.get_parameter('logger').get_parameter_value().bool_value
        self.timer_period = self.get_parameter('timer').get_parameter_value().double_value
        self.save_image = self.get_parameter('save_image').get_parameter_value().bool_value
        self.save_interval = self.get_parameter('save_interval').get_parameter_value().double_value

        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )
        
        self.br = CvBridge()
        self.cap = None
        self.last_save_time = None
        self.saved_image_count = 0
        self.save_dir = None

        if self.save_image:
            self.save_dir = self._prepare_save_directory()
            self.get_logger().info('Saving captured images to: %s' % self.save_dir)
        
        if self.data_source == 'camera':
            self.cap, camera_source = self._open_camera_capture()
            if self.cap is None:
                self.get_logger().error('Cannot open camera device: %s' % camera_source)
                rclpy.shutdown()
                sys.exit(1)
            self.get_logger().info('Opened camera source: %s' % camera_source)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        elif self.data_source == 'video':
            self.cap = cv2.VideoCapture(self.video_path)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            if not self.cap.isOpened():
                self.get_logger().error('Cannot open video file: %s' % self.video_path)
                rclpy.shutdown()
                sys.exit(1)
        elif self.data_source == 'image':
            if os.path.isdir(self.img_dir):
                self.img_list = sorted(os.listdir(self.img_dir))
                self.img_num = 0
            else:
                self.get_logger().error('Not a directory file: %s' % self.img_dir)
                rclpy.shutdown()
                sys.exit(1)
        else:
            self.get_logger().error("Wrong data source: %s \nCheck that the DATA_SOURCE variable is either 'camera', 'image', or 'video'." % self.data_source)
            rclpy.shutdown()
            sys.exit(1)
        self.publisher = self.create_publisher(Image, self.pub_topic, self.qos_profile)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def _open_camera_capture(self):
        attempted_sources = []
        camera_sources = []

        if self.camera_device:
            camera_sources.append(self.camera_device)
            if self.camera_device.startswith('/dev/video'):
                camera_index = self.camera_device.replace('/dev/video', '', 1)
                if camera_index.isdigit():
                    camera_sources.append(int(camera_index))
        else:
            camera_sources.append(self.cam_num)

        for source in camera_sources:
            attempted_sources.append(str(source))
            cap = cv2.VideoCapture(source)
            if cap.isOpened():
                return cap, source
            cap.release()

            if isinstance(source, (str, int)) and hasattr(cv2, 'CAP_V4L2'):
                cap = cv2.VideoCapture(source, cv2.CAP_V4L2)
                if cap.isOpened():
                    return cap, source
                cap.release()

        return None, ', '.join(attempted_sources)

    def _prepare_save_directory(self):
        current_dir = os.path.dirname(os.path.realpath(__file__))
        workspace_root = os.path.dirname(os.path.dirname(os.path.dirname(current_dir)))
        save_root = os.path.join(workspace_root, 'src', 'skk_assign', 'image_source')
        session_folder = datetime.now().strftime('%Y%m%d_%H%M%S')
        save_dir = os.path.join(save_root, session_folder)
        os.makedirs(save_dir, exist_ok=True)
        return save_dir

    def _save_frame_if_needed(self, frame):
        if not self.save_image or self.save_dir is None:
            return

        now = self.get_clock().now().nanoseconds / 1e9
        if self.last_save_time is not None and now - self.last_save_time < self.save_interval:
            return

        filename = 'frame_%06d.jpg' % self.saved_image_count
        filepath = os.path.join(self.save_dir, filename)
        if cv2.imwrite(filepath, frame):
            self.last_save_time = now
            self.saved_image_count += 1

    def timer_callback(self):
        if self.data_source == 'camera':
            ret, frame = self.cap.read()
            if ret:
                frame = cv2.resize(frame, (640, 480))
                self._save_frame_if_needed(frame)
                image_msg = self.br.cv2_to_imgmsg(frame, encoding='bgr8')
                image_msg.header = Header()
                image_msg.header.stamp = self.get_clock().now().to_msg()
                image_msg.header.frame_id = 'image_frame' 
                self.publisher.publish(image_msg)
                if self.logger:
                    cv2.imshow('Camera Image', frame)
                    cv2.waitKey(1)
        elif self.data_source == 'image':
            while self.img_num < len(self.img_list):
                img_file = self.img_list[self.img_num]
                img_path = os.path.join(self.img_dir, img_file)
                img = cv2.imread(img_path)
                if img is None:
                    self.get_logger().warn('Skipping non-image file: %s' % img_file)
                else:
                    img = cv2.resize(img, (640, 480))
                    self._save_frame_if_needed(img)
                    image_msg = self.br.cv2_to_imgmsg(img, encoding='bgr8')
                    image_msg.header = Header()
                    image_msg.header.stamp = self.get_clock().now().to_msg()
                    image_msg.header.frame_id = 'image_frame'
                    self.publisher.publish(image_msg)
                    if self.logger:
                        self.get_logger().info('Published image: %s' % img_file)
                        cv2.imshow('Saved Image', img)
                        cv2.waitKey(1)
                
                self.img_num += 1
                break
            else:
                self.img_num = 0
        elif self.data_source == 'video':
            ret, img = self.cap.read()
            if ret:
                img = cv2.resize(img, (640, 480))
                self._save_frame_if_needed(img)
                image_msg = self.br.cv2_to_imgmsg(img, encoding='bgr8')
                image_msg.header = Header()
                image_msg.header.stamp = self.get_clock().now().to_msg()
                image_msg.header.frame_id = 'image_frame'
                self.publisher.publish(image_msg)
                print(image_msg.header)
                if self.logger:
                    cv2.imshow('Video Frame', img)
                    cv2.waitKey(1)
            else:
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)  # Reset video to the first frame
    
def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nshutdown\n\n")
        pass
    node.destroy_node()
    if node.cap is not None and node.cap.isOpened():
        node.cap.release()
    cv2.destroyAllWindows()
    rclpy.shutdown()
  
if __name__ == '__main__':
    main()
