import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16, Bool 

class Cobot(Node):

    def __init__(self):
        super().__init__('cobot')
        self.range_subscription = self.create_subscription(
            Int16,
            'range',
            self.range_callback,
            10)
        self.range_subscription  # prevent unused variable warning

        self.estop_subscription = self.create_subscription(
            Bool,
            'estop',
            self.estop_callback,
            10)
        self.estop_subscription  # prevent unused variable warning
        
        self.estop = False

    def publish_speed(self, speed):
        print(speed)

    def range_callback(self, msg):
        range = msg.data
        print("Range: {}".format(range))
        if self.estop:
            return
        elif range < 400:
            self.publish_speed("STOP")
        elif range < 800:
            self.publish_speed("SLOW")
        else:
            self.publish_speed("FULL_SPEED")

    def estop_callback(self, msg):
        estop = msg.data
        print("Estop: {}".format(estop))
        
        if estop:
            self.estop = True
            self.publish_speed("STOP")
        else:
            self.estop = False
        

def main(args=None):
    rclpy.init(args=args)

    cobot = Cobot()

    rclpy.spin(cobot)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    cobot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()