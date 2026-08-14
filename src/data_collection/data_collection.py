import serial
import marshal
import types
import os
import time
import glob

real_path = os.path.dirname(os.path.realpath(__file__))
pyc = open((real_path)+'/data_collection_func_lib.cpython-310.pyc', 'rb').read()
code = marshal.loads(pyc[16:])
module = types.ModuleType('module_name')
exec(code, module.__dict__)


class _DummyVideoCapture:
    def set(self, *_args, **_kwargs):
        return False

    def read(self):
        return False, None

    def isOpened(self):
        return False

    def release(self):
        return None


def _disable_hidden_camera_capture_if_requested():
    # Let the ROS image publisher own the webcam during launch-based runs.
    if os.environ.get('SKK_DISABLE_DATA_COLLECTION_CAMERA') == '1':
        module.cv2.VideoCapture = lambda *_args, **_kwargs: _DummyVideoCapture()
        print('data_collection camera capture disabled; external ROS camera node will own the webcam.')


def _find_serial_port(default_port="/dev/ttyACM0"):
    env_port = os.environ.get('SKK_SERIAL_PORT')
    if env_port:
        return env_port

    if os.path.exists(default_port):
        return default_port

    by_id_candidates = sorted(glob.glob('/dev/serial/by-id/*'))
    if by_id_candidates:
        return by_id_candidates[0]

    tty_candidates = sorted(glob.glob('/dev/ttyACM*')) + sorted(glob.glob('/dev/ttyUSB*'))
    if tty_candidates:
        return tty_candidates[0]

    raise FileNotFoundError(
        'No serial device found. Connect the controller board or set SKK_SERIAL_PORT.'
    )

def main():
    DATA_PATH= os.path.dirname(real_path) + '/camera_perception_pkg/camera_perception_pkg/lib/Collected_Datasets' 
    CAMERA_NUM = 2
    SERIAL_PORT = _find_serial_port("/dev/ttyACM0")
    MAX_STEERING = 7  # 사용자 정의 최대 조향 단계

    print(DATA_PATH)
    _disable_hidden_camera_capture_if_requested()
    print(f'Using serial port: {SERIAL_PORT}')

    # 데이터 수집 객체 초기화
    data_collector = module.Data_Collect(path=DATA_PATH, cam_num=CAMERA_NUM, max_steering=MAX_STEERING)
    ser = serial.Serial(SERIAL_PORT, 115200, timeout=1)
    time.sleep(1)
    try:
        # 숨겨진 코드 프로세스 시작
        while True:
            # 한 번의 키보드 입력 처리
            result = data_collector.process()

            # 프로세스 종료 플래그 확인
            if result["exit"]:
                steering = 0
                left_speed = 0
                right_speed = 0
                message = f"s{steering}l{left_speed}r{right_speed}\n"
                ser.write(message.encode())
                break

            # 현재 제어 값 가져오기
            control_values = data_collector.get_control_values()

            # 시리얼 송신
            message = f"s{control_values['steering']}l{control_values['left_speed']}r{control_values['right_speed']}\n"
            ser.write(message.encode())

            # 디버깅용 출력
            print(f"Sent: {message.strip()}")

    except KeyboardInterrupt:
        steering = 0
        left_speed = 0
        right_speed = 0
        message = f"s{steering}l{left_speed}r{right_speed}\n"
        ser.write(message.encode())
        print("Program interrupted.")
    finally:
        ser.close()
        data_collector.cleanup()
        print("Serial connection closed.")

if __name__ == "__main__":
    main()
