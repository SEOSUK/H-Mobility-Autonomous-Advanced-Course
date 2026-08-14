import time
import serial
import glob
import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy
from interfaces_pkg.msg import MotionCommand
from .lib import protocol_convert_func_lib as PCFL

#---------------Variable Setting---------------
# Subscribe할 토픽 이름
SUB_TOPIC_NAME = "topic_control_signal"

# 아두이노 장치 이름 (ls /dev/ttyA* 명령을 터미널 창에 입력하여 확인)
PORT='/dev/ttyACM0'
#----------------------------------------------


def find_serial_port(default_port=PORT):
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

class SerialSenderNode(Node):
  def __init__(self, sub_topic=SUB_TOPIC_NAME):
    super().__init__('serial_sender_node')
    
    self.declare_parameter('sub_topic', sub_topic)
    self.declare_parameter('port', PORT)
    
    self.sub_topic = self.get_parameter('sub_topic').get_parameter_value().string_value
    requested_port = self.get_parameter('port').get_parameter_value().string_value
    self.serial_port = find_serial_port(requested_port)
    self.ser = serial.Serial(self.serial_port, 115200, timeout=1)
    time.sleep(1)
    self.get_logger().info(f'Using serial port: {self.serial_port}')
    
    qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, 
                             history=QoSHistoryPolicy.KEEP_LAST, 
                             durability=QoSDurabilityPolicy.VOLATILE, 
                             depth=1)
    
    self.subscription = self.create_subscription(MotionCommand, self.sub_topic, self.data_callback, qos_profile)

  def data_callback(self, msg):
    steering = msg.steering
    left_speed = msg.left_speed
    right_speed = msg.right_speed

    serial_msg =  PCFL.convert_serial_message(steering, left_speed, right_speed)
    self.ser.write(serial_msg.encode())

def main(args=None):
  rclpy.init(args=args)
  node = SerialSenderNode()
  try:
      rclpy.spin(node)
      
  except KeyboardInterrupt:
      print("\n\nshutdown\n\n")
      steering = 0
      left_speed = 0
      right_speed = 0
      message = PCFL.convert_serial_message(steering, left_speed, right_speed)
      node.ser.write(message.encode())
      pass
    
  finally:
    if hasattr(node, 'ser'):
      node.ser.close()
    print('closed')
    
  node.destroy_node()
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()
