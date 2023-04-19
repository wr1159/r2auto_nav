# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool

import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
LimitSwitchPin = 21
GPIO.setup(LimitSwitchPin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

#publisher to publish if limit swtich is detected or not
class SwitchPublisher(Node):

    def __init__(self):
        super().__init__('switch_pub')
        self.publisher_ = self.create_publisher(Bool, 'limit_switch', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)


    def timer_callback(self):
        msg = Bool()
        if GPIO.input(LimitSwitchPin) == 0:
            msg.data = True
        else:
            msg.data = False
        self.get_logger().info(str(msg.data))
        self.publisher_.publish(msg)
        


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = SwitchPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
