#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
class MyNode(Node):#node class

    def __init__(self):
        super().__init__("first_node")
        self.get_logger().info("Hello from ros2")

def main(args=None):
    #charecteristic of the code
    rclpy.init(args=args)#initialize
    node = MyNode()#node from the class
    rclpy.spin(node)#mahes it run without stopping
    
    ##end of the session
    rclpy.shutdown()

if __name__== '__main__':
    main()