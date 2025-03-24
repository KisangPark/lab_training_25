"""
specifications

get map (occupancy grid) and current pose (posestamped)
apply A* algorithm to matrix
given path position info, generate orientation information by differenciate (difference -> )

publish as path
"""

import rclpy
from rclpy.node import Node
import numpy as np
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseWithCovariance
import math
from geometry_msgs.msg import Quaternion
from tf_transformations import quaternion_from_euler



"""A* algorithm"""
import heapq

def heuristic(a, b):
    return abs(b[0] - a[0]) + abs(b[1] - a[1])

def a_star_path(matrix_map, start, goal):
    rows, cols = matrix_map.shape
    
    def get_neighbors(pos):
        r, c = pos
        neighbors = []
        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nr, nc = r + dr, c + dc
            if 0 <= nr < rows and 0 <= nc < cols:
                if matrix_map[nr, nc] != 100:
                    neighbors.append((nr, nc))
            else:
                # Allow exploration outside the current map
                neighbors.append((nr, nc))
        return neighbors

    frontier = [(0, start)]
    came_from = {}
    cost_so_far = {start: 0}

    while frontier:
        current_cost, current = heapq.heappop(frontier)

        if current == goal:
            break

        for next_node in get_neighbors(current):
            new_cost = cost_so_far[current] + 1
            if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                cost_so_far[next_node] = new_cost
                priority = new_cost + heuristic(goal, next_node)
                heapq.heappush(frontier, (priority, next_node))
                came_from[next_node] = current

    path = []
    current = goal
    while current != start:
        path.append(current)
        current = came_from.get(current)
        if current is None:
            return None  # No path found
    path.append(start)
    path.reverse()

    return path




class PathGenerator(Node):
    def __init__(self):
        super().__init__('path_generator')
        
        #intializing subscription and publication
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10)
        self.pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/pose',
            self.pose_callback,
            10)
        self.path_publisher = self.create_publisher(Path, '/path', 10)
        self.occupancy_grid = None
        self.current_pose = None

        self.goal_pose = PoseStamped()  # goal pose! below -> need to define
        self.goal_pose.pose.position.x = 19.0
        self.goal_pose.pose.position.y = 0.0
        quat = quaternion_from_euler(0, 0, 0)
        self.goal_pose.pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
        
        self.timer = self.create_timer(1.0, self.generate_and_publish_path)
        
        

    #callbacks -> saving
    def map_callback(self, msg):
        self.occupancy_grid = msg
    def pose_callback(self, msg): #get covariancestamped
        self.current_pose = msg.pose

    def grid_to_world(self, grid_x, grid_y):
        world_x = grid_x * self.occupancy_grid.info.resolution + self.occupancy_grid.info.origin.position.x
        world_y = grid_y * self.occupancy_grid.info.resolution + self.occupancy_grid.info.origin.position.y
        return world_x, world_y
        

    def generate_path(self):
        if self.occupancy_grid is None or self.current_pose is None:
            return None
        
        # occupancy grid into numpy array
        grid = np.array(self.occupancy_grid.data).reshape(
            (self.occupancy_grid.info.height, self.occupancy_grid.info.width)
        )
        #values -1 or 0, so add 1 to each element!
        grid += np.ones((self.occupancy_grid.info.height, self.occupancy_grid.info.width), dtype=np.uint8)

        #if grid is 0x0, rebuild the size by 10x10 and start
        if self.occupancy_grid.info.height == 0 or self.occupancy_grid.info.width== 0:
            self.get_logger().info("map initialize with 0, remake") # at start, 0
            grid = np.zeros((20,20))
        
        
        # positions -> get position from current position and goal
        start = (int(self.current_pose.pose.position.x), int(self.current_pose.pose.position.y))
        goal = (int(self.goal_pose.pose.position.x), int(self.goal_pose.pose.position.y))

        # use A* path planning algorithm defined above
        #path -> array of list
        #use grid, start, goal
        if self.occupancy_grid:
            self.get_logger().info("grid1: %d" %grid[0][0]) # map is valid

        path = a_star_path(grid, start, goal)
        
        if path is None:
            self.get_logger().warn("no path")
            return None
        

        # Convert path to ROS Path message
        path_msg = Path()
        path_msg.header = self.occupancy_grid.header
        
        for i in range(len(path)-1):
            pose = PoseStamped()
            pose.header = self.occupancy_grid.header
            #1st assign position value -> transform
            pose.pose.position.x, pose.pose.position.y = self.grid_to_world(float(path[i][0]), float(path[i][1]))
            next_x, next_y = self.grid_to_world(float(path[i+1][0]), float(path[i+1][1]))
            #pose.pose.position.x = float(path[i][0])
            #pose.pose.position.y = float(path[i][1])
            #self.get_logger().info("path info1: %d" %path[i][0])
            #self.get_logger().info("path info2: %d" %path[i][1]) #path info correct, robot going to weird direction..?
            
            # oreintation acaluation -> differenciate and into quaternion
            if i < len(path) - 1:
                dx = next_x - pose.pose.position.x
                dy = next_x - pose.pose.position.y
                #dx = path[i+1][0] - path[i][0] #path is standard
                #dy = path[i+1][1] - path[i][1]
                yaw = math.atan2(dy, dx)
                #self.get_logger().info("yaw: %f" %yaw)# yaw = 0, correct
            else:
                # For the last point, use the orientation of the previous segment
                yaw = math.atan2(path[i][1] - path[i-1][1], path[i][0] - path[i-1][0])
            
            # Convert yaw to quaternion
            q = quaternion_from_euler(0, 0, yaw)
            #self.get_logger().info("quaternion 0: %f" %q[0])
            #self.get_logger().info("quaternion 1: %f" %q[1])
            #self.get_logger().info("quaternion 2: %f" %q[2])
            #self.get_logger().info("quaternion 3: %f" %q[3])
            pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]) #ros quaternion message
            
            path_msg.poses.append(pose)
        
        return path_msg
    
    def generate_and_publish_path(self):
        path = self.generate_path()
        if path is not None:
            self.path_publisher.publish(path)

def main(args=None):
    rclpy.init(args=args)
    path_generator = PathGenerator()
    rclpy.spin(path_generator)
    path_generator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
