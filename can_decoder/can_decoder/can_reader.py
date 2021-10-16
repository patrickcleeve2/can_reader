import rclpy
from rclpy.node import Node

import serial
from can_msgs.msg import Frame


class CAN_Reader(Node):
    def __init__(self, SERIAL_PORT: str = "/dev/ttyACM0", DBC_FILE: str = None):
        super().__init__("can_reader")

        self.SERIAL_PORT = SERIAL_PORT
        self.can_publisher = self.create_publisher(Frame, "mini/can", 10)

    def run(self):

        ser = serial.Serial(self.SERIAL_PORT)
        ser.flushInput()

        while True:  # TODO: change to not shutdown or spin?
            try:
                ser_bytes = ser.readline()

                decoded_bytes = ser_bytes.decode("utf-8", errors="ignore").split(",")

                if len(decoded_bytes) == 11:  # only parse full messages

                    frame = str(decoded_bytes[0])
                    message_id = int(decoded_bytes[1], base=16)
                    message_data = [int(num, base=16) for num in decoded_bytes[2:-1]]

                    self.get_logger().info(f"{message_id}: {message_data}")

                    msg = Frame()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.id = message_id
                    msg.dlc = len(message_data)
                    msg.data = message_data  # can data is now in base 10?

                    self.can_publisher.publish(msg)

            except KeyboardInterrupt:
                print("Keyboard Interrupt")
                break


def main(args=None):
    rclpy.init(args=args)

    can_reader = CAN_Reader()
    can_reader.run()

    can_reader.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
