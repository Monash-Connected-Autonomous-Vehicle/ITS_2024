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
from std_msgs.msg import String

class CrossIntersectionPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        topic_name = '/behaviour_planner/out'
        self.publisher_ = self.create_publisher(String, topic_name, 10)
        self.out()

    def out(self):
        """
        Outputs behaviour planner command to path planner (STOP or GO)
        """
        # Scenario variables
        collision_detected = True
        intersection_leader = True
        road_leader = False

        self.stop()

        # Collision detection
        if collision_detected:
            if intersection_leader:
                # Publish AskPermission message to other vehicles

                # Subscribe to AskPermission acknowledgements from other vehicles

                # Cross intersection
                pass
            else:
                if road_leader:
                    # Subscribe to AskPermission

                    # Publish AskPermission acknowledgement

                    # Subscribe to AllowPass (becomes intersection leader)

                    # Publish AllowPass acknowledgement
                    pass
                else:
                    # Subscribe to HDOV
                    pass
            # Move to edge of intersection and stop
            pass
        else:
            # Cross intersection
            self.go()

    def go(self):
        msg = String()
        msg.data = 'GO'
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: {msg}'.format(msg=msg.data))

    def stop(self):
        msg = String()
        msg.data = 'STOP'
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: {msg}'.format(msg=msg.data))


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = CrossIntersectionPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
