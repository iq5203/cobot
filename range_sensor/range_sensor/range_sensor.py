import random

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16

class RangePublisher(Node):
  # Publishes range data produced by a generator every second to ROS2 topic
  # "range"

  def __init__(self, range_generator):
      # range_generator should be a generator that takes 0 arguments and
      # produces 16 bit integers
      super().__init__('range_sensor')
      self.publisher_ = self.create_publisher(Int16, 'range', 10)

      # create timer that will call timer_callback every 1 second
      timer_period = 1.0  # seconds
      self.timer = self.create_timer(timer_period, self.timer_callback)
      self.range_generator = range_generator

  def timer_callback(self):
  # Uses generator provided from constructor to generate a new range value and 
  # publishes it to ROS2 topic "range"
    msg = Int16()
    data = next(self.range_generator, None)
    if data is not None:
        msg.data = data
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
    else:
        print("Data done")

def make_range():
    # returns a random range value between 0 and 1000
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
