import rclpy
import requests
from rclpy.node import Node
from example_interfaces.msg import String
from queue import Queue


class NovelPubNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.get_logger().info("NovelPubNode has been started")
        self.novels_queue = Queue()
        self.novel_pub =  self.create_publisher(String, "My_novel", 10)
        self.timer = self.create_timer(2, self.timer_callback)
    def timer_callback(self):
        if self.novels_queue.empty():
            self.get_logger().info("novels_queue is empty")
        else:
            line = self.novels_queue.get()
            msg = String()
            msg.data = line
            self.novel_pub.publish(msg)
            self.get_logger().info("publish novel: {}".format(msg))
    def download_novel(self, url):
        response = requests.get(url)
        response.encoding = "utf-8"
        text = response.text
        self.get_logger().info("finish downloading from {}, total {}".format(url, len(text)))
        for line in text.splitlines():
            self.novels_queue.put(line)
def main():
    rclpy.init()
    node = NovelPubNode("My_novel_pub")
    node.download_novel("http://0.0.0.0:8080/novel1.txt")
    rclpy.spin(node)
    rclpy.shutdown()