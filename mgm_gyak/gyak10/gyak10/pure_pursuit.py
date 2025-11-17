import rclpy
from rclpy.node import Node
import copy
import math

from geometry_msgs.msg import PoseStamped, Twist, Pose
from nav_msgs.msg import Odometry, Path
from tf_transformations import euler_from_quaternion
from visualization_msgs.msg import MarkerArray, Marker

import tf2_ros
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose_stamped

class PURE_PURSUIT_CONTROL(Node):
    def __init__(self):
        super().__init__('pure_pursuit')

        self.declare_parameter("wheelbase", 0.3)
        self.wheelbase = self.get_parameter("wheelbase").value 
        self.declare_parameter("lookahead_distance", 0.3)
        self.lookahead_distance = self.get_parameter("lookahead_distance").value

        self.declare_parameter('max_speed', 0.1)
        self.max_speed = self.get_parameter('max_speed').value
        self.declare_parameter('max_steering_speed', 0.6)
        self.max_steering_speed = self.get_parameter('max_steering_speed').value

        self.path = Path()
        self.pose_actual = Pose()

        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)

        self.dt = 0.1

        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", 1)
        self.pub_viz = self.create_publisher(MarkerArray, "/viz_closest", 1)

        self.sub_path = self.create_subscription(Path, "/path", self.callback_path, 1)
        # self.sub_odom = self.create_subscription(Odometry, "/odom", self.callback_odom, 1)

        self.timer = self.create_timer(self.dt, self.timer_callback)

    def callback_path(self, msg: PoseStamped):
        self.path = msg

    def callback_odom(self, msg: Odometry):
        self.pose_actual = msg.pose.pose

    def timer_callback(self):
        # if not self.path or not self.odom or len(self.path.poses) < 2:
        if not self.path  or len(self.path.poses) < 2:
            return
        
        # Get actual pose
        if self.tfBuffer.can_transform("map", "base_link", rclpy.time.Time(), rclpy.duration.Duration(seconds=0.1)):
            trans_base2map = self.tfBuffer.lookup_transform("map", "base_link", rclpy.time.Time())
            base = PoseStamped()
            base.pose.orientation.w = 1.0

            self.pose_actual = do_transform_pose_stamped(base, trans_base2map)

        else:
            return

        # Closest point
        index_closest_point = 0
        closest_distance = 9999.0
        for index in range(len(self.path.poses)):
            path_point = self.path.poses[index]
            dx_cal = path_point.pose.position.x - self.pose_actual.pose.position.x
            dy_cal = path_point.pose.position.y - self.pose_actual.pose.position.y
            distance_cal = math.sqrt(dx_cal**2 + dy_cal**2)
            if distance_cal < closest_distance:
                index_closest_point = index
                closest_distance = distance_cal
        
        closest_point = self.path.poses[index_closest_point]
        self.send_point(closest_point, "closest_point")



        # Lookahead point
        lookahead_index = index_closest_point + 1
        while lookahead_index < len(self.path.poses):
            lookahead_point = self.path.poses[lookahead_index]
            dx = lookahead_point.pose.position.x - self.pose_actual.pose.position.x
            dy = lookahead_point.pose.position.y - self.pose_actual.pose.position.y
            distance = math.sqrt(dx**2 + dy**2)
            if distance > self.lookahead_distance:
                break
            lookahead_index +=1

        lookahead_point = self.path.poses[lookahead_index]
        self.send_point(lookahead_point, "lookahead_point")


        # Pure pursuit 

                # Pure pursuit control
        dx = lookahead_point.pose.position.x - self.pose_actual.pose.position.x
        dy = lookahead_point.pose.position.y - self.pose_actual.pose.position.y
        linear_error = math.sqrt(dx**2 + dy**2)

        target_yaw = math.atan2(dy, dx)
        q = [self.pose_actual.pose.orientation.x,
             self.pose_actual.pose.orientation.y,
             self.pose_actual.pose.orientation.z,
             self.pose_actual.pose.orientation.w]
        current_yaw = euler_from_quaternion(q)[2]
        # Calculate angular error 
        angular_error = target_yaw - current_yaw

        ## Longitudinal control
        linear_output = 0.0
        if linear_error > self.max_speed:
            linear_output = self.max_speed
        elif linear_error > 0.1:
            linear_output = linear_error

        ## Lateral control
        corrected_angle = math.atan2(math.sin(angular_error), math.cos(angular_error))

        steering_angle = math.atan(2 * self.wheelbase * math.sin(corrected_angle)/ linear_error)

        # Limit angular output
        steering_angle = max(min(steering_angle, self.max_steering_speed), -self.max_steering_speed)

        # Publish control commands
        cmd = Twist()
        cmd.linear.x = linear_output
        cmd.angular.z = steering_angle
        self.pub_cmd.publish(cmd)




    def send_point(self, closest_point:PoseStamped, ns:str):
        # Create visualization markers
        marker_array = MarkerArray()
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = closest_point.pose
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.lifetime = rclpy.duration.Duration(seconds=0.1).to_msg()
        marker_array.markers.append(marker)

        self.pub_viz.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    pid_control_ = PURE_PURSUIT_CONTROL()
    rclpy.spin(pid_control_)
    pid_control_.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()