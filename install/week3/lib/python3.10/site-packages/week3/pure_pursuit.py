"""
pseudo code
    1) receive path
    2) point by point look ahead
    3) calculate angular velocity from linear velocity, yaw, and distance
    4) publish into cmd_vel

"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from math import atan2, cos, sin, sqrt, pow

class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')
        self.path_subscription = self.create_subscription(
            Path,
            '/user_path',
            self.path_follow,
            10)
        self.robot_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.save_pose,
            10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.linear_velocity = 0.5
        self.min_lookahead = 0.5


    def save_pose(self, msg):
        self.current_pos = msg.pose.pose.position #object
        #translate quaternion into euler -> z axis angle yaw
        q = msg.pose.pose.quaternion
        self.yaw = atan2(2.0*(q.y*q.z + q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z)
        self.get_logger().info("pose saved")

        #yaw and current pose updated every receiving action


    def path_follow(self, msg):
        #generally once path is converted
        path = msg.poses #path is list of posestamped -> element.pose.position.x
        self.get_logger().info("path arrived, controlling...")

        for pose in path: #for elements in path
            path_point = pose.pose.position #include x, y, z

            #from here, calculate!
            
            while True:
                try:
                    temp_x = self.current_pos.x
                    temp_y = self.current_pos.y
                    yaw = self.yaw
                except:
                    temp_x = 0.0
                    temp_y = 0.0
                    yaw = 0.0
                #1) ld value
                dx = path_point.x - temp_x
                dy = path_point.y - temp_y
                ld = sqrt(dx*dx + dy*dy)
                #2) alpha value
                temp_angle = atan2(dy, dx)
                alpha = temp_angle - yaw
                #3) radius
                R = ld/(2*sin(alpha))
                # finally, angular velocity
                self.angular_velocity = self.linear_velocity / R

                twist = Twist()
                twist.linear.x = self.linear_velocity
                twist.angular.z = self.angular_velocity * 3 #heuristic modified
                self.cmd_vel_publisher.publish(twist)

                #terminalize condition
                #keep the distance! using ld break
                #if close enough -> break

                if ld < self.min_lookahead:
                    break
            self.get_logger().info("next point")
            

def main(args=None):
    rclpy.init(args=args)
    path_follower = PathFollower()
    rclpy.spin(path_follower)
    path_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
