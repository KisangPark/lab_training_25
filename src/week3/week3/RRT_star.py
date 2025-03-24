"""
pseudo code

get costmap from ros2 navigation (occupancy grid? cost cloud?)
-> know from turtle bot

got origin, map, goal
    - origin & map already given
    - if goal point is given by user topic -> path generate pixel
    - modify with resolution & make user path topic

save costmap -> ros2 run
subscribe costmap, data reshape & save to image

"""

import random
import heapq
import os
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path #Float32MultiArray
from geometry_msgs.msg import Point, PoseStamped
from rclpy.qos import QoSProfile


"""1. define node for rrt star"""
class NODE: #class for RRT node
    def __init__(self, point):
        self.point = point
        self.parent = None
        self.cost = 0.0 #float cost


"""2. drawing function"""
def draw_path(map_image, points):
    #get image, path points -> draw line between each pair!
    for i in range(len(points) - 1):
        cv2.line(map_image, points[i], points[i+1], (0,255,0), 2)
    return map_image



"""
3. functions for RRT
    1) generate random point
    2) get nearest tree vertex
    3) action input -> also named as steer
    4) find Xnear, check cost and update connection (parent)
    5) reverse connection update (child)
    6) collision avoidance?

here, functions about
- generate random
- nearest & Xnear
- steering
"""

def generate_random_point(img, goal, goal_sample_rate):
    #generate random sample from the map
    #get goal according to the sample rate -> biasing

    if random.randint(0, 100) > goal_sample_rate:
        return (random.randint(0, img.shape[1]), random.randint(0, img.shape[0]))
        #tuple of positions -> globally random
    else:
        return goal

def distance(point1, point2): #calculate distance between two points
    #points are 2 dimensional -> 2d norm to get distance
    return np.linalg.norm(np.array(point1) - np.array(point2))

def nearest_node(nodes, random_point):
    #finding nearest node
    #return node of smallest distance, using lambda key
    return min(nodes, key=lambda node: distance(node.point, random_point))

def Xnear(current, tree_nodes, dist):
    #return tree nodes within the distance
    #current and tree_nodes for node object
    #current means xnew after steering
    Xnear = []
    for node in tree_nodes:
        if distance(current.point, node.point) <= dist:
            Xnear.append(node)
    return Xnear #list of nodes

def steer(tree_node, random_point, control_input):
    #get node, random point, control_input
    #get new node
    #progress forward with fixed value
    if distance(tree_node.point, random_point) < control_input:
        return NODE(random_point)

    else: #toward fixed value!
        #get two point, difference to get direction
        #add direction * distance to start point!
        start = np.array(tree_node.point)
        toward = np.array(random_point) - start
        length = np.linalg.norm(toward)
        unit_direction = np.array(toward / length)
        xnew = start + control_input * unit_direction
        #now, xnew is numpy array with point
        #change into integer & tuple
        new_pos = tuple(xnew.astype(int))
        return NODE(new_pos) #.astype(int)


"""4. collision check"""
#return true or false

def line_collision_check(first_point, second_point, img):
    #check collision in map
    #check if there is point occupied between two points

    x_values = [first_point[0], second_point[0]]
    y_values = [first_point[1], second_point[1]]
    x_values.sort() #values in increasing order
    y_values.sort()
    for x in range(x_values[0], x_values[1]+1):
        for y in range(y_values[0], y_values[1]+1):
            if img[y, x] > 0:  # occupied value, black is occupied (or 100?)
                return False
    return True

    #currently, detecting all objects within the square


"""
5. connection update
    1) pick parent
    2) update near nodes (forward & backward)
"""
#pick parent & forward
#after choosing xnew with steering, compare costs within Xnear

def choose_parent(current, Xnear, image):
    #choose lowest cost within Xnear

    #if no nodes in Xnear, return none (no parent)
    if not Xnear:
        return None

    #if nodes exist, compare costs
    min_cost = float('inf')
    for node in Xnear: #near nodes? -> by Xnear function
        if line_collision_check(node.point, current.point, image): #true if no collision
            cost = node.cost + distance(node.point, current.point) #cost for distance
            #if using other costs, I can modify here to make changes!

            if cost < min_cost:
                min_cost = cost
                current.parent = node
                current.cost = cost
    return current #if new_node.parent else None


#backward rewiring
#parent already chosen from upper function
#-> need current node and Xnear, compare Xnear costs and rewire!

def rewire(current, Xnear, image):

    for node in Xnear:
        if node != current.parent: #no need to deal with parent
            if line_collision_check(node.point, current.point, image) and current.cost + distance(current.point, node.point) < node.cost:
                node.parent = current
                node.cost = current.cost + distance(current.point, node.point)



"""
6. RRT search -> overall !
    1) get image, start, goal, iteration, goal sample rate (percent)
    2) get random point
    3) collision check -> steer
    4) Xnear
    5) choose parent with X near
    6) rewire
"""

def RRT_STAR_search(img, start, goal, iter_max=5000, goal_sample_rate=20):

    #1. define node list
    nodes = [NODE(start)]
    
    for i in range(iter_max):
        #get random, nearest, xnew
        random_point = generate_random_point(img, goal, goal_sample_rate)
        nearest = nearest_node(nodes, random_point)
        new_node = steer(nearest, random_point, control_input=1) #30? change with resolution
        #print(new_node)

        #if not collide
        if line_collision_check(nearest.point, new_node.point, img):
            
            #get near set
            near_nodes = Xnear(new_node, nodes, dist=3)#here, Xnear, 60, distance=0.2
            #update parent
            new_node = choose_parent(new_node, near_nodes, img)
            #if parent connected
            if new_node:
                nodes.append(new_node)
                rewire(new_node, near_nodes, img)

        #if collide -> no action -> no append

        #terminalize
        if distance(new_node.point, goal) <= 1:  # 목표에 가까워진 경우 종료
            return get_path(new_node)



def get_path(node):
    #extract path
    path = []
    while node:
        path.append(node.point)
        node = node.parent
    return path[::-1]  # reverse order


## for test
def draw_point(map_image):
    #draw point
    cv2.line(map_image, (30,0), (125,80), (0,255,0), 10)
    return map_image


"""
pseudo code

got origin, map, goal
    - origin & map already given
    - if goal point is given by user topic -> path generate pixel
    - modify with resolution & make user path topic
"""

class RRT_STAR(Node):
    def __init__(self):
        super().__init__('rrt_star')
        qos_profile = QoSProfile(depth=10)

        self.start = (25,68)
        self.flag = 0

        #get map and initial position instead of subscription
        self.subscription_1 = self.create_subscription(
            OccupancyGrid,
            '/map', # length 3 vector of integer angles
            self.save_map,
            qos_profile
        )
        self.subscription_1 #prevent unused variable warning

        self.subscription_2 = self.create_subscription(
            Point, #PoseStamped,
            '/user_goal', # length 3 vector of integer angles
            self.create_path,
            qos_profile
        )
        self.subscription_2 #prevent unused variable warning

        #publisher
        self.publisher = self.create_publisher(Path,
        'user_path', qos_profile)

        #execute once?
        self.timer = self.create_timer(1, self.publish_path)



    def save_map(self, msg):
        array = np.array(msg.data)
        reshaped = np.reshape(array,(117,125)) #into pixel image
        self.map = reshaped

    def create_path(self, msg):
        #translate: get float goal number, into pixel
        x_goal_pixel = int((1.21 - msg.x) / 0.05)
        y_goal_pixel = int((3.42 - msg.y) / 0.05)
        goal_point = (x_goal_pixel, y_goal_pixel)


        #create path -> using upper methods
        self.path = RRT_STAR_search(self.map, self.start, goal_point)
        #path returned in pixel wise
        #now, resolution & publish by path
        self.get_logger().info("path generated")
        #print(len(self.path)) #here -> path is not fully generated
        self.get_logger().info("path length: %d" %len(self.path))


    def publish_path(self):
        path_final = Path()
        path_final.header.frame_id = 'map'

        #self path is in pixel wise
        try:
            if self.path:
                #self.get_logger().info("path exists -> 1st done")
                for point in self.path:
                    stamped = PoseStamped()
                    #got tuple in point
                    real_x = 1.21 - point[0]*0.05
                    real_y = 3.42 - point[1]*0.05

                    stamped.pose.position.x = real_x
                    #self.get_logger().info("path: %f" %stamped.pose.position.x)
                    stamped.pose.position.y = real_y
                    #header?
                    stamped.header.frame_id = 'map'

                    path_final.poses.append(stamped) #temp_list
                    #self.get_logger().info("correctly added")
                    #self.get_logger().info("position calculated -> 2nd done")

                    if len(self.path) > 5 and self.flag == 0:
                        self.flag = 1

        except:
            self.get_logger().info("no path generated yet")
        
        #path_final.poses = temp_list

        #self.get_logger().info("path: %f" %stamped.pose.position.x)
        #self.get_logger().info('successfully transitioned, publishing...')
        if self.flag == 1:
            self.publisher.publish(path_final)
            flag = 2
        else:
            pass



def main(args=None):
    #main function call
    rclpy.init(args=args)
    node = RRT_STAR()
    rclpy.spin(node) #spin or spin_once
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    """main function"""
    main()