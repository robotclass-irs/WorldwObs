#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObjectAvoidanceNode(Node):
    def __init__(self):
        super().__init__('object_avoidance_node')
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.lidar_callback,
            10)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.safe_distance = 1  # Meters
        self.get_logger().info('Object Avoidance Node Started')

    def lidar_callback(self, msg):
        ranges = msg.ranges
        front_distance = ranges[179]
        left_distance = ranges[269]
        right_distance = ranges[89]
        print(right_distance)
       # print("haha")

        twist_msg = Twist()

        if front_distance < self.safe_distance:
            if right_distance < self.safe_distance:
            	twist_msg.linear.x = 0.0
            	twist_msg.angular.z = 0.5
            elif left_distance < self.safe_distance:
            	twist_msg.linear.x = 0.0
            	twist_msg.angular.z = -0.5
            else:
            	twist_msg.linear.x = 0.05
            	twist_msg.angular.z = 0.5
        elif right_distance < self.safe_distance:
            	twist_msg.linear.x = 0.05
            	twist_msg.angular.z = 0.5
        elif left_distance < self.safe_distance:
            	twist_msg.linear.x = 0.2
            	twist_msg.angular.z = -0.5
        else:
            # No obstacle detected, move forward
            twist_msg.linear.x = 0.2
            twist_msg.angular.z = 0.0

        self.publisher.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectAvoidanceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
