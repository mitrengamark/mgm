import rclpy
from rclpy.node import Node
import copy
import math

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion

class PID_CONTROL(Node):
    def __init__(self):
        super().__init__('control_node')

        self.declare_parameter('Kp', 1.0)
        self.declare_parameter('Ki', 0.0)
        self.declare_parameter('Kd', 0.0)
        self.Kp = self.get_parameter('Kp').value
        self.Ki = self.get_parameter('Ki').value
        self.Kd = self.get_parameter('Kd').value

        self.declare_parameter('max_speed', 0.1)
        self.max_speed = self.get_parameter('max_speed').value
        self.declare_parameter('max_angular_speed', 0.3)
        self.max_angular_speed = self.get_parameter('max_angular_speed').value

        self.target_pose = PoseStamped()
        self.odom = Odometry()

        self.integral = 0.0
        self.previous_error = 0.0
        self.dt = 0.1

        self.sub_target = self.create_subscription(PoseStamped, "/goal_pose", self.callback_target, 1)
        self.sub_odom = self.create_subscription(Odometry, "/odom", self.callback_odom, 1)
        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", 1)
        self.timer = self.create_timer(self.dt, self.timer_callback)

    def callback_target(self, msg: PoseStamped):
        self.target_pose = msg

    def callback_odom(self, msg: Odometry):
        self.odom = msg

    def timer_callback(self):
        if not self.target_pose or not self.odom:
            return

        # Compute PID control signals
        dx = self.target_pose.pose.position.x - self.odom.pose.pose.position.x
        dy = self.target_pose.pose.position.y - self.odom.pose.pose.position.y
        linear_error = math.sqrt(dx**2 + dy**2)

        target_yaw = math.atan2(dy, dx)
        g_goal = [
            self.target_pose.pose.orientation.x,
            self.target_pose.pose.orientation.y,
            self.target_pose.pose.orientation.z,
            self.target_pose.pose.orientation.w]
        goal_yaw = euler_from_quaternion(g_goal)[2]
        q = [self.odom.pose.pose.orientation.x,
             self.odom.pose.pose.orientation.y,
             self.odom.pose.pose.orientation.z,
             self.odom.pose.pose.orientation.w]
        current_yaw = euler_from_quaternion(q)[2]


        ## Longitudinal control
        linear_output = 0.0
        if linear_error > self.max_speed:
            linear_output = self.max_speed
        elif linear_error > 0.1:
            linear_output = linear_error

        ## Lateral control
        # Calculate angular error with wrapping
        if linear_output > 0.0:
            angular_error = target_yaw - current_yaw
        else:
            angular_error = goal_yaw - current_yaw

        corrected_angle = math.atan2(math.sin(angular_error), math.cos(angular_error))

        # Update integral and derivative terms
        self.integral += corrected_angle * self.dt
        derivative = (corrected_angle - self.previous_error) / self.dt

        # Compute control output
        angular_output = self.Kp * corrected_angle + self.Ki * self.integral + self.Kd * derivative 

        # Limit angular output
        angular_output = max(min(angular_output, self.max_angular_speed), -self.max_angular_speed)

        # Publish control commands
        cmd = Twist()
        cmd.linear.x = linear_output
        cmd.angular.z = angular_output
        self.pub_cmd.publish(cmd)

        self.previous_error = linear_error

def main(args=None):
    rclpy.init(args=args)
    pid_control_ = PID_CONTROL()
    rclpy.spin(pid_control_)
    pid_control_.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()