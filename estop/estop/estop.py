import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import random

class EstopPublisher(Node):
    def __init__(self, estop_generator):
        super().__init__('estop')
        self.publisher_ = self.create_publisher(Bool, 'estop', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.estop_generator = estop_generator

    def timer_callback(self):
        bool_msg = Bool()
        data = next(self.estop_generator, None)
        if data != None:
            bool_msg.data = data     
            self.publisher_.publish(bool_msg)
            self.get_logger().info('Publishing estop: "%s"' % str(bool_msg.data))
        else:
            print("Data done")

def make_estop():
    while True:
        if random.randint(0, 10) == 0:
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
