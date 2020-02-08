#!/usr/bin/env python

# Copyright (C) 2012 Jon Stephan.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.


import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist


class TwistToMotors(Node):
    """
    twist_to_motors - converts a twist message to motor commands.  Needed for navigation stack

    """

    def __init__(self):
        super(TwistToMotors, self).__init__("twist_to_motors")
        self.nodename = "twist_to_motors"
        self.get_logger().info("%s started" % self.nodename)

        self.w = self.declare_parameter("base_width", 0.2).value

        self.pub_lmotor = self.create_publisher(Float32, 'lwheel_vtarget', 10)
        self.pub_rmotor = self.create_publisher(Float32, 'rwheel_vtarget', 10)
        self.create_subscription(Twist, 'twist', self.twist_callback, 10)

        self.rate = self.declare_parameter("rate", 50).value
        self.timeout_ticks = self.declare_parameter("timeout_ticks", 2).value
        self.left = 0
        self.right = 0

    def spin(self):
        r = self.create_rate(self.rate)
        idle = self.create_rate(10)
        then = self.get_clock().now()
        self.ticks_since_target = self.timeout_ticks

        ###### main loop  ######
        while rclpy.ok():

            while rclpy.ok() and self.ticks_since_target < self.timeout_ticks:
                self.spin_once()
                r.sleep()
            idle.sleep()

    def spin_once(self):
        # dx = (l + r) / 2
        # dr = (r - l) / w

        self.right = 1.0 * self.dx + self.dr * self.w / 2
        self.left = 1.0 * self.dx - self.dr * self.w / 2
        # rospy.loginfo("publishing: (%d, %d)", left, right) 

        self.pub_lmotor.publish(self.left)
        self.pub_rmotor.publish(self.right)

        self.ticks_since_target += 1

    def twist_callback(self, msg):
        # rospy.loginfo("-D- twistCallback: %s" % str(msg))
        self.ticks_since_target = 0
        self.dx = msg.linear.x
        self.dr = msg.angular.z
        self.dy = msg.linear.y


def main(args=None):
    rclpy.init(args=args)
    try:
        twist_to_motors = TwistToMotors()
        twist_to_motors.spin()
    except rclpy.exceptions.ROSInterruptException:
        pass

    twist_to_motors.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
