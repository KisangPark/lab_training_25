import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped, PoseWithCovariance
from nav_msgs.msg import Path
import math

class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.path_subscription = self.create_subscription(Path, 'path', self.path_callback, 10)
        self.current_pose_subscription = self.create_subscription(PoseWithCovarianceStamped, 'pose', self.pose_callback, 10)
        
        self.path = None
        self.current_pose = None
        self.current_goal = None
        self.goal_threshold = 0.1  # meters
        
        self.timer = self.create_timer(0.1, self.follow_path)

    def path_callback(self, msg): # first element as current goal -> move straight into first goals
        self.path = msg.poses #list of posestamped
        self.current_goal = self.path.pop(0).pose if self.path else None

    def pose_callback(self, msg): #save current pose
        self.current_pose = msg.pose.pose

    def follow_path(self): #timer publisher callback, control robot
        if not self.path or not self.current_pose or not self.current_goal:
            return

        dx = self.current_goal.position.x - self.current_pose.position.x
        dy = self.current_goal.position.y - self.current_pose.position.y
        distance = math.sqrt(dx*dx + dy*dy) #heuristic distance between current goal and robot

        if distance < self.goal_threshold: #if within threshold, update goal and end to move to next goal
            self.current_goal = self.path.pop(0).pose if self.path else None
            return

        # if not within, move
        angle = math.atan2(dy, dx)
        current_angle = 2 * math.atan2(self.current_pose.orientation.z, self.current_pose.orientation.w)
        self.get_logger().info("current angle: %f" %current_angle) #fine... sign?
        angle_diff = angle - current_angle

        linear_speed = 0.2  # Adjust as needed
        angular_speed = 0.1 * angle_diff  # Adjust as needed

        cmd_vel = Twist()
        cmd_vel.linear.x = linear_speed
        cmd_vel.angular.z = angular_speed

        self.cmd_vel_publisher.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    path_follower = PathFollower()
    rclpy.spin(path_follower)
    path_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
