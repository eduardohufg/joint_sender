import rclpy 
from std_msgs.msg import * 
from rclpy.node import Node 
import serial
import signal
import sys

def my_map(value: float, in_min: float, in_max,out_min: float, out_max: float) -> float:

    targetPos: float = (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    return targetPos

def get_min_max_encoder_pos(zero_encoder_pos: float, min_angle_pos: float, max_angle_pos: float) -> tuple:
    current_180 = 2011.0

    min_encoder_pos: float = min_angle_pos * current_180 / 180.0 + zero_encoder_pos
    max_encoder_pos: float = max_angle_pos * current_180 / 180.0 + zero_encoder_pos

    return min_encoder_pos, max_encoder_pos
        

class Sender_uart(Node):
    def __init__(self):
        super().__init__('Senser_uart')
        
        self.joint_2= self.create_subscription(Float64, '/arm_teleop/joint2', self.callback_joint2, 10)
        self.joint_2
        self.angle_joint2: float = 0.0
        self.port: str = "/dev/ttyTHS1"
        self.baud_rate: int = 1000000 
        try:
            self.uart = serial.Serial(self.port, self.baud_rate, timeout=1)
            message: str = "init_uart\n"
            self.uart.write(message.encode())
            print(f"Uart initialized")
        except serial.SerialException as e:
            self.uart = None
            print(f"Error to initialized UART: {e}")

        #self.timer = self.create_timer(0.05, self.joint_arm_2)

        signal.signal(signal.SIGINT, self.close_uart)
        

    def callback_joint2(self, msg):
        self.angle_joint2 = float(msg.data)
        self.joint_arm_2()

    def close_uart(self, signum, frame):
        if self.uart:
            print("Closing UART connection...")
            message: str = "close_uart\n"
            self.uart.write(message.encode())
            self.uart.close()
        sys.exit(0)

    def joint_arm_2(self):

        if self.uart is None:
            return
        
        # Angle limits for joint 2
        zero_encoder_pos: float = 2500.0
        min_angle_pos: float = -10.0
        max_angle_pos: float = 180.0

        if (self.angle_joint2 >= min_angle_pos and self.angle_joint2 <= max_angle_pos):
   
            min_encoder_pos, max_encoder_pos = get_min_max_encoder_pos(zero_encoder_pos, min_angle_pos, max_angle_pos)

            targetPos: float = my_map(self.angle_joint2, min_angle_pos, max_angle_pos, min_encoder_pos, max_encoder_pos)
            message: str = f"{targetPos}\n"
            self.uart.write(message.encode())
            print(f"Target Real Angle: {self.angle_joint2}")
            print(f"Target Encoder Angle {targetPos}")
        else:
            print(f"Angle out of range: {self.angle_joint2}")

def main(args=None):
    rclpy.init(args=args)
    listener=Sender_uart()
    try:
        rclpy.spin(listener)
    except KeyboardInterrupt:
        pass
    finally:
        listener.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()