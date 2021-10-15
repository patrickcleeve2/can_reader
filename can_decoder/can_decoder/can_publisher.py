import rclpy
from rclpy.node import Node

import cantools
from can_msgs.msg import Frame

class CAN_Decoder(Node):

    def __init__(self, DBC_FILE: str) -> None:
        super().__init__('can_decoder')


        self.db = cantools.database.load_file(
            DBC_FILE,
            strict=False,
        )

        self.can_data = {}
        self.total_msgs = 162#self.config["total_messages"]

        self.can_subscriber = self.create_subscription(
            Frame, "mini/can", self.can_callback, 10
        )

    def can_callback(self, msg: Frame) -> None:

        message_id = msg.id
        message_data = list(msg.data)

        message = self.db.decode_message(message_id, message_data)

        # self.get_logger().info(f"{message}")

        self.can_data.update(message)

        if (len(self.can_data) == self.total_msgs):  # only publish once all msgs are received
            self.get_logger().info(
                f"starting to publish: {len(self.can_data)}/{self.total_msgs}"
            )
            self.publish_state()
        else:
            self.get_logger().info(
                f"waiting on msgs: {len(self.can_data)}/{self.total_msgs}"
            )
    def publish_state(self):

        self.get_logger().info(f"{self.can_data}")

def main(args=None):
    rclpy.init(args=args)

        # # TODO: relative path based on package location..
    can_decoder = CAN_Decoder(
        DBC_FILE="/home/patrick/dev_ws/src/can_reader/opendbc/hyundai_kia_generic.dbc"
        )
    rclpy.spin(can_decoder)

    can_decoder.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()