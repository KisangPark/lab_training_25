"""
saving costmap

save costmap -> ros2 run
subscribe costmap, data reshape & save to image
"""


import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from nav2_msgs.msg import Costmap #Float32MultiArray
from rclpy.qos import QoSProfile


class SAVE(Node):
    def __init__(self):
        super().__init__('save_costmap')
        qos_profile = QoSProfile(depth=10)

        #subscriber
        self.subscription = self.create_subscription(
            Costmap,
            '/global_costmap/costmap_raw', # length 3 vector of integer angles
            self.save,
            qos_profile
        )
        self.subscription #prevent unused variable warning



    def save(self, msg):
        array = np.array(msg.data)
        reshaped = np.reshape(array,(117,125))
        #cv2.imshow('test', reshaped)
        #cv2.waitKey(0)
        cv2.imwrite('costmap.pgm', reshaped)
        #cv2.destroyAllWindows()


def main(args=None):
    #main function call
    rclpy.init(args=args)
    node = SAVE()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    """main function"""
    main()