import rclpy
from rclpy.node import Node

class Bage2Video(Node):
    def __init__(self):
        super().__init__('bag_to_video')
        self.get_logger().info("Node started")

def main():
    rclpy.init()
    node = Bage2Video()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()