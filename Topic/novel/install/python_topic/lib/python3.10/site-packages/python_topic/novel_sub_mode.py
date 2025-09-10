import espeakng
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String
from queue import Queue

import threading
import time


class NovelSubNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.queue = Queue()
        self.subscription = self.create_subscription(String, "My_novel", self.listener_callback, 10)
        self.speaker = espeakng.Speaker()
        self.speaker.voice = "en-us"
        self.speaker.rate = 150
        self.speaker.volume = 100
        self.speaker.pitch = 50

        self.speaker_thread = threading.Thread(target=self.speak_it)
        self.speaker_thread.start()

    def listener_callback(self, msg):
        self.get_logger().info("I heard: " + msg.data)
        self.speaker.say(msg.data)

    def speak_it(self):
        speaker = espeakng.Speaker()
        speaker.voice = "zh"
        while rclpy.ok():
            if not self.queue.empty():
                text = self.queue.get()
                self.get_logger().info("I say: " + text)
                speaker.say(text)
                speaker.wait()
            else:
                time.sleep(1)

def main():
    rclpy.init()
    node = NovelSubNode("novel_sub_node")
    rclpy.spin(node)
    rclpy.shutdown()