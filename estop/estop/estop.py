import random

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class EstopPublisher(Node):
# Takes a generator which should return a boolean, and publishes the values 
# provided to ROS2 topic "estop" with a period of 1 second

  def __init__(self, estop_generator):
    super().__init__('estop')
    self.publisher_ = self.create_publisher(Bool, 'estop', 10)
    timer_period = 1.0  # seconds

    # create timer that will call timer_callback every 1 second
    self.timer = self.create_timer(timer_period, self.timer_callback)
    self.estop_generator = estop_generator

  def timer_callback(self):
  # Uses generator provided from constructor to generate a new estop value and 
  # publishes it to ROS2 topic "estop"

    bool_msg = Bool()
    data = next(self.estop_generator, None)
    if data is not None:
        bool_msg.data = data     
        self.publisher_.publish(bool_msg)
        self.get_logger().debug(f'Publishing estop: "{bool_msg.data}"')
    else:
        self.get_logger().warn("Data done")
        print("Data done")

def make_estop():
    # This estop generate generates random estop values with a 20% probability.
    while True:
        if random.randint(0, 10) < 2:
            yield True
        else:
            yield False

def main(args=None):
    rclpy.init(args=args)
    generator = make_estop()
    estop_publisher = EstopPublisher(generator)

    rclpy.spin(estop_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    estop_publisher.destroy_node()
    rclpy.shutdown()
