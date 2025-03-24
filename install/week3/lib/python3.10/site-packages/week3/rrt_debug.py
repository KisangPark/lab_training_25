"""
make path from the image & just import to ROS
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
    #get nearest node, random point, control_input
    #get new node
    #progress forward with fixed value
    if distance(tree_node.point, random_point) < control_input:
        node = NODE(random_point)
        #node.parent = tree_node
        return node

    else: #toward fixed value!
        #get two point, difference to get direction
        #add direction * distance to start point!
        start = np.array(tree_node.point)
        toward = np.array(random_point) - start
        #print(toward) #coordinate OK
        length = np.linalg.norm(toward)
        unit_direction = np.array(toward / length)
        #print(unit_direction)#OK
        dislocation = control_input * unit_direction
        print("dislocation:", dislocation[0]) #dislocation, correctly calculated
        xnew = start + dislocation
        print("starting:", start[0])
        print("new wnode position:", xnew[0])
        
        #now, xnew is numpy array with point
        #change into integer & tuple
        x_value = round(xnew[0])
        y_value = round(xnew[1])
        new_pos = (x_value, y_value) #use round instead of astype
        node = NODE(new_pos)
        #node.parent = tree_node
        return node #.astype(int)

def bresenham_line(x1, y1, x2, y2): #return the list of points
    #get two points (integer values)
    #1) return longer axis
    #2) move pixel by pixel, check with the intermediate point
    points = []
    dx = abs(x2 - x1) #number of pixels
    dy = abs(y2 - y1)

    #adder -> determine the sign
    #easier think -> make order between x1 and x2
    sx = 1 if x1 < x2 else -1
    sy = 1 if y1 < y2 else -1
    err = dx - dy

    while True:
        points.append((x1, y1)) #x1 and y1 moving
        if x1 == x2 and y1 == y2:
            break

        #err2: two times difference of dx and dy
        err2 = err * 2
        if err2 > -dy:
            err -= dy
            x1 += sx
        if err2 < dx:
            err += dx
            y1 += sy

    return points

def line_collision_check(point1, point2, image):
    #get two points, 

    # Get the coordinates of the line between the two points
    line_points = bresenham_line(point1[0], point1[1], point2[0], point2[1])
    
    # Check pixel values along the line
    for (x, y) in line_points:
        # Ensure coordinates are within image bounds
        if 0 <= x < image.shape[1] and 0 <= y < image.shape[0]:
            if image[y, x] == 0:  # Check if the pixel is not zero
                return False
    
    return True

"""
def line_collision_check(first_point, second_point, img):
    #check collision in map
    #check if there is point occupied between two points

    x_values = [first_point[0], second_point[0]]
    y_values = [first_point[1], second_point[1]]
    x_values.sort() #values in increasing order
    y_values.sort()
    for x in range(x_values[0], x_values[1]+1):
        for y in range(y_values[0], y_values[1]+1):
            if img[y, x] == 0:  # occupied value, black is occupied (or 100?) >
                return False
    return True

    #currently, detecting all objects within the square
"""

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
for debugging: draw all actions
"""
# cv2.line(img, random_point, nearest.point, (0,255,0), 1)
        # resized_image = cv2.resize(img, (500,468))
        # cv2.imshow('processing', resized_image)
        # cv2.waitKey(100) 

def RRT_STAR_debug(img, start, goal, iter_max=5000, goal_sample_rate=20):

    #1. define node list
    nodes = [NODE(start)]

    #change image into RGB -> only draw on this image!
    rgb_img = cv2.cvtColor(img,cv2.COLOR_GRAY2RGB)
    
    for i in range(iter_max):
        #get random, nearest, xnew
        random_point = generate_random_point(img, goal, goal_sample_rate)
        nearest = nearest_node(nodes, random_point)

        #1st drawing: random point
        cv2.line(rgb_img, random_point, nearest.point, (255,0,0), 1)
        resized_image = cv2.resize(rgb_img, (500,468))
        cv2.imshow('processing', resized_image)
        cv2.waitKey(100) 

        new_node = steer(nearest, random_point, control_input=1) #30? change with resolution

        #2nd drawing: after steering
        cv2.line(rgb_img, new_node.point, nearest.point, (0,255,0), 2)
        resized_image = cv2.resize(rgb_img, (500,468))
        cv2.imshow('processing', resized_image)
        cv2.waitKey(100) 

        #if not collide
        if line_collision_check(nearest.point, new_node.point, img):
            
            #get near set
            near_nodes = Xnear(new_node, nodes, dist=7)#here, Xnear, 60, distance=0.2
            #update parent
            new_node = choose_parent(new_node, near_nodes, img)
            #if parent connected
            if new_node:
                nodes.append(new_node)
                rewire(new_node, near_nodes, img)

            #3rd drawing: after collision check, updated
            cv2.line(rgb_img, new_node.parent.point, new_node.point, (0,0,255), 2)
            resized_image = cv2.resize(rgb_img, (500,468))
            cv2.imshow('processing', resized_image)
            cv2.waitKey(100) 

        #if collide -> no action -> no append

        #dead zone?

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

def makepath_from_image():
    img = cv2.imread('/home/kisangpark/map.pgm', cv2.IMREAD_UNCHANGED)
    print('width=',img.shape[1],' height=',img.shape[0])
    # 시작점과 목적지 설정
    # 1000,1000  => 1270, 600
    start_point = (25,68)  # 시작점 좌표
    goal_point = (95,30)  # 목적지 좌표

    #path = RRT_STAR_search(img, start_point, goal_point)
    path = RRT_STAR_debug(img, start_point, goal_point)

    return path #in pixel

"""
pseudo code

got origin, map, goal
    - origin & map already given
    - if goal point is given by user topic -> path generate pixel
    - modify with resolution & make user path topic
"""

class RRT_STAR2(Node):
    def __init__(self):
        super().__init__('rrt_star_2')
        qos_profile = QoSProfile(depth=10)

        #no subscription, get path from image
        self.path = makepath_from_image()
        print(self.path)

        #publisher
        self.publisher = self.create_publisher(Path,
        'user_path', qos_profile)

        #execute once?
        self.timer = self.create_timer(1, self.publish_path)


    def publish_path(self):
        #pixel path to real number path
        #real path to ros path
        path_final = Path()
        path_final.header.frame_id = 'map'
        temp_list = []

        if self.path:
            #self.get_logger().info("path exists -> 1st done")
            for point in self.path:
                x = point[0]*0.05 - 1.21
                y = 3.42 - point[1]*0.05
                #now, add real number to path topic
                stamped = PoseStamped()
                stamped.pose.position.x = x
                stamped.pose.position.y = y
                #header?
                stamped.header.frame_id = 'map'

                path_final.poses.append(stamped) #temp_list
                temp_list.append((x,y))
            
            #print(temp_list)
                #self.get_logger().info("correctly added")
                #self.get_logger().info("position calculated -> 2nd done")

                #if len(self.path) > 5 and self.flag == 0:
                #    self.flag = 1

        
        #path_final.poses = temp_list

        #self.get_logger().info("path: %f" %stamped.pose.position.x)
        #self.get_logger().info('successfully transitioned, publishing...')
        #if self.flag == 1:
        self.publisher.publish(path_final)
        #    flag = 2
        #else:
        #    pass



def main(args=None):
    #main function call
    rclpy.init(args=args)
    node = RRT_STAR2()
    rclpy.spin(node) #spin or spin_once
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    """main function"""
    main()