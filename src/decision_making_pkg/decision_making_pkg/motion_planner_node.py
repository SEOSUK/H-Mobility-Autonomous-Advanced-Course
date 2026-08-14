import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy
from rclpy.duration import Duration

from std_msgs.msg import String, Bool
from interfaces_pkg.msg import PathPlanningResult, DetectionArray, MotionCommand
from .lib import decision_making_func_lib as DMFL

#---------------Variable Setting---------------
SUB_DETECTION_TOPIC_NAME = "detections"
SUB_PATH_TOPIC_NAME = "path_planning_result"
SUB_TRAFFIC_LIGHT_TOPIC_NAME = "yolov8_traffic_light_info"
SUB_LIDAR_OBSTACLE_TOPIC_NAME = "lidar_obstacle_info"
PUB_TOPIC_NAME = "topic_control_signal"

#----------------------------------------------

# 모션 플랜 발행 주기 (초) - 소수점 필요 (int형은 반영되지 않음)
TIMER = 0.1
PATH_TIMEOUT = 0.5
STEERING_LIMIT = 7
STEERING_KP = 0.08
STEERING_KI = 0.00
STEERING_KD = 0.02
STEERING_D_FILTER_CUTOFF_HZ = 1.5
STEERING_INTEGRAL_LIMIT = 30.0
STOP_DELAY_SECONDS = 4.0 # green 높이 정지 기준이 만족된 뒤 몇 초 후에 멈출지.
TRANSLATIONAL_SMOOTHING_ALPHA = 0.8 # 병진 스무딩
TRANSLATIONAL_SPEED_GAIN = 2.5 # 병진 목표 속도 gain
RED_HOLD_SECONDS = 1.0
GREEN_STOP_HEIGHT_THRESHOLD_START = 60.0
GREEN_STOP_HEIGHT_THRESHOLD = 80.0
OBSTACLE_STOP_HEIGHT_ARM_THRESHOLD = 120.0 # obstacle pixel gate
GREEN_SLOWDOWN_MIN_SPEED_RATIO = 0.2
CRUISE_BASE_SPEED = 100
STOP_CLASSES = {'obstacle', 'green'}

class MotionPlanningNode(Node):
    def __init__(self):
        super().__init__('motion_planner_node')

        # 토픽 이름 설정
        self.sub_detection_topic = self.declare_parameter('sub_detection_topic', SUB_DETECTION_TOPIC_NAME).value
        self.sub_path_topic = self.declare_parameter('sub_lane_topic', SUB_PATH_TOPIC_NAME).value
        self.sub_traffic_light_topic = self.declare_parameter('sub_traffic_light_topic', SUB_TRAFFIC_LIGHT_TOPIC_NAME).value
        self.sub_lidar_obstacle_topic = self.declare_parameter('sub_lidar_obstacle_topic', SUB_LIDAR_OBSTACLE_TOPIC_NAME).value
        self.pub_topic = self.declare_parameter('pub_topic', PUB_TOPIC_NAME).value
        
        self.timer_period = self.declare_parameter('timer', TIMER).value
        self.path_timeout = self.declare_parameter('path_timeout', PATH_TIMEOUT).value
        self.steering_limit = self.declare_parameter('steering_limit', STEERING_LIMIT).value
        self.steering_kp = self.declare_parameter('steering_kp', STEERING_KP).value
        self.steering_ki = self.declare_parameter('steering_ki', STEERING_KI).value
        self.steering_kd = self.declare_parameter('steering_kd', STEERING_KD).value
        self.steering_d_filter_cutoff_hz = self.declare_parameter(
            'steering_d_filter_cutoff_hz',
            STEERING_D_FILTER_CUTOFF_HZ,
        ).value
        self.steering_integral_limit = self.declare_parameter(
            'steering_integral_limit',
            STEERING_INTEGRAL_LIMIT,
        ).value
        self.stop_delay_seconds = self.declare_parameter(
            'stop_delay_seconds',
            STOP_DELAY_SECONDS,
        ).value
        self.translational_smoothing_alpha = self.declare_parameter(
            'translational_smoothing_alpha',
            TRANSLATIONAL_SMOOTHING_ALPHA,
        ).value
        self.translational_speed_gain = self.declare_parameter(
            'translational_speed_gain',
            TRANSLATIONAL_SPEED_GAIN,
        ).value
        self.red_hold_seconds = self.declare_parameter(
            'red_hold_seconds',
            RED_HOLD_SECONDS,
        ).value
        self.green_stop_height_threshold_start = self.declare_parameter(
            'green_stop_height_threshold_start',
            GREEN_STOP_HEIGHT_THRESHOLD_START,
        ).value
        self.green_stop_height_threshold = self.declare_parameter(
            'green_stop_height_threshold',
            GREEN_STOP_HEIGHT_THRESHOLD,
        ).value
        self.obstacle_stop_height_arm_threshold = self.declare_parameter(
            'obstacle_stop_height_arm_threshold',
            OBSTACLE_STOP_HEIGHT_ARM_THRESHOLD,
        ).value
        self.green_slowdown_min_speed_ratio = self.declare_parameter(
            'green_slowdown_min_speed_ratio',
            GREEN_SLOWDOWN_MIN_SPEED_RATIO,
        ).value

        # QoS 설정
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        # 변수 초기화
        self.detection_data = None
        self.path_data = None
        self.last_path_time = None
        self.traffic_light_data = None
        self.lidar_data = None

        self.steering_command = 0
        self.left_speed_command = 0
        self.right_speed_command = 0
        self.last_logged_command = None
        self.steering_error_integral = 0.0
        self.prev_steering_error = 0.0
        self.steering_derivative_input_history = [0.0, 0.0]
        self.steering_derivative_output_history = [0.0, 0.0]
        self.steering_d_filter_coefficients = (1.0, 0.0, 0.0, 0.0, 0.0)
        self.current_left_speed = 0.0
        self.current_right_speed = 0.0
        self.red_last_seen_time = None
        self.red_stop_active = False
        self.drive_started = False
        self.stop_sequence_armed = False
        self.stop_sequence_armed_reason = None
        self.stop_delay_start_time = None
        self.stop_delay_reason = None
        self.permanent_stop = False
        self.permanent_stop_reason = None
        self.green_slowdown_active = False
        self.obstacle_stop_height_seen = False
        self.last_logged_obstacle_height = None
        self.configure_steering_derivative_filter()

        # 서브스크라이버 설정
        self.detection_sub = self.create_subscription(DetectionArray, self.sub_detection_topic, self.detection_callback, self.qos_profile)
        self.path_sub = self.create_subscription(PathPlanningResult, self.sub_path_topic, self.path_callback, self.qos_profile)
        self.traffic_light_sub = self.create_subscription(String, self.sub_traffic_light_topic, self.traffic_light_callback, self.qos_profile)
        self.lidar_sub = self.create_subscription(Bool, self.sub_lidar_obstacle_topic, self.lidar_callback, self.qos_profile)

        # 퍼블리셔 설정
        self.publisher = self.create_publisher(MotionCommand, self.pub_topic, self.qos_profile)

        # 타이머 설정
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def detection_callback(self, msg: DetectionArray):
        self.detection_data = msg

    def path_callback(self, msg: PathPlanningResult):
        if not msg.x_points or not msg.y_points:
            self.path_data = None
            self.last_path_time = None
            return

        self.path_data = list(zip(msg.x_points, msg.y_points))
        self.last_path_time = self.get_clock().now()
                
    def traffic_light_callback(self, msg: String):
        self.traffic_light_data = msg

    def lidar_callback(self, msg: Bool):
        self.lidar_data = msg

    def has_recent_path(self):
        if self.path_data is None or self.last_path_time is None:
            return False

        age = self.get_clock().now() - self.last_path_time
        return age <= Duration(seconds=float(self.path_timeout))

    def get_detected_classes(self):
        if self.detection_data is None:
            return set()

        return {
            detection.class_name
            for detection in self.detection_data.detections
            if detection.class_name
        }

    def reset_steering_pid(self):
        self.steering_error_integral = 0.0
        self.prev_steering_error = 0.0
        self.steering_derivative_input_history = [0.0, 0.0]
        self.steering_derivative_output_history = [0.0, 0.0]

    def configure_steering_derivative_filter(self):
        dt = float(self.timer_period)
        if dt <= 0.0:
            dt = TIMER

        sample_rate_hz = 1.0 / dt
        nyquist_hz = 0.5 * sample_rate_hz
        cutoff_hz = float(self.steering_d_filter_cutoff_hz)

        if cutoff_hz <= 0.0:
            self.get_logger().warn(
                'steering_d_filter_cutoff_hz must be positive. D-term filter bypassed.'
            )
            self.steering_d_filter_coefficients = (1.0, 0.0, 0.0, 0.0, 0.0)
            return

        if cutoff_hz >= nyquist_hz:
            clamped_cutoff_hz = max(1e-6, 0.99 * nyquist_hz)
            self.get_logger().warn(
                f'steering_d_filter_cutoff_hz={cutoff_hz:.3f} exceeds Nyquist '
                f'({nyquist_hz:.3f} Hz). Clamping to {clamped_cutoff_hz:.3f} Hz.'
            )
            cutoff_hz = clamped_cutoff_hz

        warped_frequency = math.tan(math.pi * cutoff_hz / sample_rate_hz)
        sqrt_two = math.sqrt(2.0)
        normalization = 1.0 / (
            1.0 + sqrt_two * warped_frequency + warped_frequency * warped_frequency
        )

        b0 = warped_frequency * warped_frequency * normalization
        b1 = 2.0 * b0
        b2 = b0
        a1 = 2.0 * (
            warped_frequency * warped_frequency - 1.0
        ) * normalization
        a2 = (
            1.0 - sqrt_two * warped_frequency + warped_frequency * warped_frequency
        ) * normalization
        self.steering_d_filter_coefficients = (b0, b1, b2, a1, a2)

    def apply_steering_derivative_filter(self, derivative):
        b0, b1, b2, a1, a2 = self.steering_d_filter_coefficients
        x1, x2 = self.steering_derivative_input_history
        y1, y2 = self.steering_derivative_output_history

        filtered_derivative = (
            b0 * derivative
            + b1 * x1
            + b2 * x2
            - a1 * y1
            - a2 * y2
        )

        self.steering_derivative_input_history = [derivative, x1]
        self.steering_derivative_output_history = [filtered_derivative, y1]
        return filtered_derivative

    def get_max_detection_height(self, class_name):
        if self.detection_data is None:
            return None

        matched_heights = [
            float(detection.bbox.size.y)
            for detection in self.detection_data.detections
            if detection.class_name == class_name
        ]
        if not matched_heights:
            return None

        return max(matched_heights)

    def compute_steering_command(self, target_slope):
        error = float(target_slope)
        dt = float(self.timer_period)
        if dt <= 0.0:
            dt = TIMER

        self.steering_error_integral += error * dt
        self.steering_error_integral = max(
            -float(self.steering_integral_limit),
            min(float(self.steering_integral_limit), self.steering_error_integral),
        )

        derivative = (error - self.prev_steering_error) / dt
        filtered_derivative = self.apply_steering_derivative_filter(derivative)
        steering_output = (
            float(self.steering_kp) * error
            + float(self.steering_ki) * self.steering_error_integral
            + float(self.steering_kd) * filtered_derivative
        )
        self.prev_steering_error = error

        steering_output = max(
            -float(self.steering_limit),
            min(float(self.steering_limit), steering_output),
        )
        return int(round(steering_output))

    def smooth_speed_command(self, current_speed, target_speed):
        alpha = max(0.0, min(1.0, float(self.translational_smoothing_alpha)))
        return current_speed + alpha * (float(target_speed) - current_speed)

    def apply_speed_targets(self, target_left_speed, target_right_speed, bypass_smoothing=False):
        if bypass_smoothing:
            self.current_left_speed = float(target_left_speed)
            self.current_right_speed = float(target_right_speed)
        else:
            self.current_left_speed = self.smooth_speed_command(
                self.current_left_speed,
                target_left_speed,
            )
            self.current_right_speed = self.smooth_speed_command(
                self.current_right_speed,
                target_right_speed,
            )

        self.left_speed_command = int(round(self.current_left_speed))
        self.right_speed_command = int(round(self.current_right_speed))

    def compute_translational_speed(self, base_speed):
        speed = float(base_speed) * float(self.translational_speed_gain)
        speed = max(0.0, min(255.0, speed))
        return int(round(speed))

    def apply_green_stop_slowdown(self, cruise_speed):
        if self.permanent_stop or not self.stop_sequence_armed:
            self.green_slowdown_active = False
            return cruise_speed

        green_height = self.get_max_detection_height('green')
        if green_height is None:
            self.green_slowdown_active = False
            return cruise_speed

        start_threshold = float(self.green_stop_height_threshold_start)
        stop_threshold = float(self.green_stop_height_threshold)
        min_speed_ratio = max(0.0, min(1.0, float(self.green_slowdown_min_speed_ratio)))

        if green_height <= start_threshold:
            self.green_slowdown_active = False
            return cruise_speed

        if stop_threshold <= start_threshold:
            speed_scale = min_speed_ratio
        else:
            progress = (green_height - start_threshold) / (stop_threshold - start_threshold)
            progress = max(0.0, min(1.0, progress))
            speed_scale = 1.0 - progress * (1.0 - min_speed_ratio)

        slowed_speed = int(round(float(cruise_speed) * speed_scale))
        slowed_speed = max(0, min(int(cruise_speed), slowed_speed))

        if not self.green_slowdown_active:
            self.get_logger().info(
                'Green stop slowdown active: '
                f'height={green_height:.1f}, start={start_threshold:.1f}, '
                f'stop={stop_threshold:.1f}, target_speed={slowed_speed}'
            )
        self.green_slowdown_active = True
        return slowed_speed

    def has_green_signal(self, detected_classes):
        if self.traffic_light_data is not None and self.traffic_light_data.data.lower() == 'green':
            return True

        return 'green' in detected_classes

    def has_red_signal(self, detected_classes):
        if self.traffic_light_data is not None and self.traffic_light_data.data.lower() == 'red':
            return True

        return 'red' in detected_classes

    def update_red_stop_state(self, detected_classes):
        now = self.get_clock().now()
        raw_red_detected = self.has_red_signal(detected_classes)

        if raw_red_detected:
            self.red_last_seen_time = now
            if not self.red_stop_active:
                self.red_stop_active = True
                self.get_logger().info('Red light detected. Holding stop.')
            return True

        if self.red_last_seen_time is None:
            self.red_stop_active = False
            return False

        elapsed_since_red = (now - self.red_last_seen_time).nanoseconds / 1e9
        if elapsed_since_red <= float(self.red_hold_seconds):
            if not self.red_stop_active:
                self.red_stop_active = True
                self.get_logger().info('Red light hold active.')
            return True

        self.red_last_seen_time = None
        if self.red_stop_active:
            self.get_logger().info('Red light hold cleared.')
        self.red_stop_active = False
        return False

    def update_start_state(self, should_stop_for_red):
        if self.drive_started:
            return True

        if should_stop_for_red:
            return False

        self.drive_started = True
        self.get_logger().info('Starting lane following.')
        return True

    def latch_permanent_stop(self, reason):
        if self.permanent_stop:
            return

        self.permanent_stop = True
        self.permanent_stop_reason = reason
        self.stop_delay_start_time = None
        self.stop_delay_reason = None
        self.get_logger().warn(f'Permanent stop latched by {reason}.')

    def update_stop_sequence(self, detected_classes):
        if self.permanent_stop or not self.drive_started:
            return False

        now = self.get_clock().now()
        obstacle_height = self.get_max_detection_height('obstacle')
        obstacle_height_threshold = float(self.obstacle_stop_height_arm_threshold)

        if obstacle_height is None:
            self.last_logged_obstacle_height = None
        elif self.last_logged_obstacle_height != obstacle_height:
            print(f"obstacle height={obstacle_height:.1f}", flush=True)
            self.last_logged_obstacle_height = obstacle_height

        if (
            not self.obstacle_stop_height_seen
            and obstacle_height is not None
            and obstacle_height > obstacle_height_threshold
        ):
            self.obstacle_stop_height_seen = True
            self.get_logger().info(
                'Obstacle stop gate armed by obstacle height: '
                f'{obstacle_height:.1f}>{obstacle_height_threshold:.1f}'
            )

        if self.stop_sequence_armed:
            green_height = self.get_max_detection_height('green')
            if green_height is None or green_height <= float(self.green_stop_height_threshold):
                return False

            trigger_reason = (
                f"green_height>{float(self.green_stop_height_threshold):.1f}"
                f"({green_height:.1f}) armed_by={self.stop_sequence_armed_reason}"
            )
            if self.stop_delay_start_time is None:
                self.stop_delay_start_time = now
                self.stop_delay_reason = trigger_reason
                self.get_logger().info(
                    f'Stop delay started: {trigger_reason}, delay={float(self.stop_delay_seconds):.1f}s'
                )
                return False

            elapsed = (now - self.stop_delay_start_time).nanoseconds / 1e9
            if elapsed >= float(self.stop_delay_seconds):
                self.latch_permanent_stop(trigger_reason)
                return True

            return False

        if self.obstacle_stop_height_seen and 'green' in detected_classes:
            self.stop_sequence_armed = True
            self.stop_sequence_armed_reason = (
                f'obstacle_height_seen>{obstacle_height_threshold:.1f}_and_green_detected'
            )
            self.get_logger().info(
                'Stop sequence armed: green detected after obstacle height gate was met.'
            )

        return False
        
    def timer_callback(self):
        detected_classes = self.get_detected_classes()
        should_stop_for_red = self.update_red_stop_state(detected_classes)
        can_drive = self.update_start_state(should_stop_for_red)
        should_stop_for_trigger = self.update_stop_sequence(detected_classes)
        target_left_speed = 0
        target_right_speed = 0
        bypass_speed_smoothing = False

        if self.permanent_stop or should_stop_for_trigger or should_stop_for_red:
            self.reset_steering_pid()
            self.steering_command = 0
            target_left_speed = 0
            target_right_speed = 0

        elif not can_drive:
            self.reset_steering_pid()
            self.steering_command = 0
            target_left_speed = 0
            target_right_speed = 0
            bypass_speed_smoothing = True

        elif not self.has_recent_path():
            self.reset_steering_pid()
            self.steering_command = 0
            target_left_speed = 0
            target_right_speed = 0
            bypass_speed_smoothing = True

        else:
            target_slope = DMFL.calculate_slope_between_points(self.path_data[-10], self.path_data[-1])
            self.steering_command = self.compute_steering_command(target_slope)
            speed_command = self.compute_translational_speed(CRUISE_BASE_SPEED)
            speed_command = self.apply_green_stop_slowdown(speed_command)

            target_left_speed = speed_command  # 예시 속도 값 (255가 최대 속도)
            target_right_speed = speed_command  # 예시 속도 값 (255가 최대 속도)

        self.apply_speed_targets(
            target_left_speed,
            target_right_speed,
            bypass_smoothing=bypass_speed_smoothing,
        )

        current_command = (
            self.steering_command,
            self.left_speed_command,
            self.right_speed_command,
        )
        if current_command != self.last_logged_command:
            print(
                f"CMD s={self.steering_command:+d} l={self.left_speed_command:03d} r={self.right_speed_command:03d}",
                flush=True,
            )
            self.last_logged_command = current_command

        # 모션 명령 메시지 생성 및 퍼블리시
        motion_command_msg = MotionCommand()
        motion_command_msg.steering = self.steering_command
        motion_command_msg.left_speed = self.left_speed_command
        motion_command_msg.right_speed = self.right_speed_command
        self.publisher.publish(motion_command_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MotionPlanningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nshutdown\n\n")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
