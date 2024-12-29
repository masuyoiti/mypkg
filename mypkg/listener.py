import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ResourceListener(Node):
    def __init__(self):
        super().__init__('resource_listener')
        self.subscription = self.create_subscription(
            String,
            'system_resources',
            self.listener_callback,
            10)
        self.subscription  # サブスクリプション変数を保持

    def listener_callback(self, msg):
        self.get_logger().info(f"Received: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = ResourceListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

