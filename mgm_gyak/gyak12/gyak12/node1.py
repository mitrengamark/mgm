import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

class ZhPub(Node):
    def __init__(self):
        super().__init__('zh_node')

        self.subscription = self.create_subscription(Odometry,'/odom',self.callback_odom,1)
        
        self.publisher_odom = self.create_publisher(Odometry, '/poz', 1)
        self.publisher_dist = self.create_publisher(Float64, '/elmozdulas', 1)

        # Contains the distance traveled by the robot
        self.dist_msg = Float64()
        # Contains the last message received
        self.last_msg = Point()
        
        self.get_logger().info('ZH Publisher node has been started')

    def callback_odom(self, msg: Odometry):
        elmozdulas = math.sqrt(
            (msg.pose.pose.position.x - self.last_msg.x)**2 + 
            (msg.pose.pose.position.y - self.last_msg.y)**2
        )
        
        if math.floor(self.dist_msg.data) < math.floor(self.dist_msg.data + elmozdulas):
            self.publisher_odom.publish(msg)
        
        self.dist_msg.data += elmozdulas
        self.publisher_dist.publish(self.dist_msg)

        self.last_msg = msg.pose.pose.position



def main(args=None):
    rclpy.init(args=args)
    zh_pub = ZhPub()
    rclpy.spin(zh_pub)
    zh_pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()