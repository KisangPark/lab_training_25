#!/usr/bin/env python3

import rospy
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib

class RobotNavigation:
    def __init__(self):
        rospy.init_node('robot_navigation')
        
        # Initialize the map
        self.map = rospy.wait_for_message('/map', OccupancyGrid)
        
        # Initialize AMCL
        rospy.wait_for_service('global_localization')
        self.global_localization = rospy.ServiceProxy('global_localization', Empty)
        self.global_localization()
        
        # Subscribe to AMCL pose estimates
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_callback)
        
        # Initialize move_base client
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()
        
        self.tf_listener = tf.TransformListener()
        
    def amcl_callback(self, msg):
        self.current_pose = msg.pose.pose
        
    def navigate_to_goal(self, x, y, theta):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation = tf.transformations.quaternion_from_euler(0, 0, theta)
        
        self.move_base_client.send_goal(goal)
        self.move_base_client.wait_for_result()
        
        if self.move_base_client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Goal reached successfully!")
        else:
            rospy.loginfo("Failed to reach the goal")

if __name__ == '__main__':
    try:
        navigator = RobotNavigation()
        
        # Define goal point
        goal_x = 2.0
        goal_y = 3.0
        goal_theta = 1.57  # 90 degrees in radians
        
        # Navigate to goal
        navigator.navigate_to_goal(goal_x, goal_y, goal_theta)
        
    except rospy.ROSInterruptException:
        pass
