import rclpy
from rclpy.node import Node
from person_msgs.msg import Person

class Listener(Node):
    def __init__(self):
        super().__init__('listener')
        self.sub = self.create_subscription(Person, 'person', self.cb, 10)

    def cb(self, msg):
        self.get_logger().info(f'Listen: {msg.name}, Age: {msg.age}')

def main(args=None):
    rclpy.init(args=args)
    node = Listener()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
