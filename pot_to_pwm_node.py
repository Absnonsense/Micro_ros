#!/usr/bin/env python3
"""ROS 2 node running on the PC.

Subscribes /pot_raw (Int32, 0..4095) from the ESP32, maps it to a PWM duty,
and publishes /pwm_cmd (Int32, 0..4095) back to the ESP32.
Higher pot voltage -> higher duty cycle (wider pulse).

Run:  python3 pot_to_pwm_node.py
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

ADC_MAX = 4095
PWM_MAX = 4095


class PotToPwm(Node):
    def __init__(self):
        super().__init__('pot_to_pwm_node')
        self.declare_parameter('smoothing', 0.3)   # 0 = none, closer to 1 = heavier filter
        self.declare_parameter('min_duty', 0)      # output limits (0..4095)
        self.declare_parameter('max_duty', PWM_MAX)

        self.filtered = None
        self.pub = self.create_publisher(Int32, 'pwm_cmd', 10)
        self.sub = self.create_subscription(Int32, 'pot_raw', self.on_pot, 10)
        self.get_logger().info('pot_to_pwm_node started')

    def on_pot(self, msg: Int32):
        s = float(self.get_parameter('smoothing').value)
        lo = int(self.get_parameter('min_duty').value)
        hi = int(self.get_parameter('max_duty').value)

        raw = max(0, min(ADC_MAX, msg.data))
        self.filtered = raw if self.filtered is None else s * self.filtered + (1.0 - s) * raw

        duty = int(lo + (hi - lo) * (self.filtered / ADC_MAX))
        duty = max(0, min(PWM_MAX, duty))

        out = Int32()
        out.data = duty
        self.pub.publish(out)
        self.get_logger().info(
            f'pot={raw:4d}  duty={duty:4d} ({100.0 * duty / PWM_MAX:5.1f}%)',
            throttle_duration_sec=0.5)


def main():
    rclpy.init()
    node = PotToPwm()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
