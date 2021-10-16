import rclpy
from rclpy.node import Node
import yaml

import cantools
from can_msgs.msg import Frame
from can_reader_msgs.msg import CarState

KPH_TO_MS = 1.0 / 3.6

class CAN_Decoder(Node):

    def __init__(self, DBC_FILE: str, CONFIG_FILE: str) -> None:
        super().__init__('can_decoder')


        self.db = cantools.database.load_file(
            DBC_FILE,
            strict=False,
        )

        self.config = self.load_config(
            CONFIG_FILE
        )

        self.can_data = {}
        self.total_msgs = 162#self.config["total_messages"]

        self.can_subscriber = self.create_subscription(
            Frame, "mini/can", self.can_callback, 10
        )

        self.state_publisher = self.create_publisher(
            CarState, "mini/vehicle_state", 10
        )

    def load_config(self, config_file):

        with open(config_file, "r") as f:
            config_dict = yaml.safe_load(f)
            self.get_logger().info(f"Config: {config_file}")

        return config_dict

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
        
        # get car velocity from wheel speeds
        wheel_speed_fl = self.can_data[self.config["wheel_speed"]["FL"]] * KPH_TO_MS
        wheel_speed_fr = self.can_data[self.config["wheel_speed"]["FR"]] * KPH_TO_MS
        wheel_speed_rl = self.can_data[self.config["wheel_speed"]["RL"]] * KPH_TO_MS
        wheel_speed_rr = self.can_data[self.config["wheel_speed"]["RR"]] * KPH_TO_MS
        v_ego = (
            wheel_speed_fl  + wheel_speed_fr + wheel_speed_rl + wheel_speed_rr
        ) / 4.0

        msg = CarState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "car"
        msg.throttle = float(self.can_data[self.config["control"]["throttle"]]) # / 254.
        msg.velocity = v_ego
        msg.steer = float(self.can_data[self.config["control"]["steer"]])
        msg.steer_rate = float(self.can_data[self.config["control"]["steer_rate"]])
        msg.brake_pressed = bool(self.can_data[
            self.config["control"]["brake_pressed"]
        ]) # nb this is not brake position, just a flag
        
        msg.yaw_rate = float(self.can_data[self.config["control"]["yaw_rate"]])
        msg.left_indicator = bool(self.can_data[self.config["indicator"]["left"]])
        msg.right_indicator = bool(self.can_data[self.config["indicator"]["right"]])


        self.state_publisher.publish(msg)

# TODO: door open, seatbelt, acc, lka, steer torque


def main(args=None):
    rclpy.init(args=args)

        # # TODO: relative path based on package location..
    can_decoder = CAN_Decoder(
        DBC_FILE="/home/patrick/dev_ws/src/can_reader/opendbc/hyundai_kia_generic.dbc",
        CONFIG_FILE="/home/patrick/dev_ws/src/can_reader/can_decoder/config/hyundai_config.yaml"
        )
    rclpy.spin(can_decoder)

    can_decoder.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()