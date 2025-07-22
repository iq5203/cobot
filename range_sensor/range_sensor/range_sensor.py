import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
import random

class RangePublisher(Node):
    def __init__(self, range_generator):
        super().__init__('range_sensor')
        self.publisher_ = self.create_publisher(Int16, 'range', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.range_generator = range_generator

    def timer_callback(self):
        msg = Int16()
        data = next(self.range_generator, None)
        if data != None:
            msg.data = data
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s"' % str(msg.data))
        else:
            print("Data done")

def make_range():
    while True:
        yield random.randint(0, 1000)

def main(args=None):
    rclpy.init(args=args)
    generator = make_range()
    range_publisher = RangePublisher(generator)

    rclpy.spin(range_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    range_publisher.destroy_node()
    rclpy.shutdown()
