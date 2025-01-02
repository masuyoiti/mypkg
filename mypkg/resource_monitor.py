import rclpy
from rclpy.node import Node
import psutil
from std_msgs.msg import String

class ResourceMonitor(Node):
    def __init__(self):
        super().__init__('resource_monitor')
        self.publisher_ = self.create_publisher(String, 'system_resources', 10)
        self.timer = self.create_timer(1.0, self.publish_resource_usage)

    def publish_resource_usage(self):
        try:
            cpu_usage = psutil.cpu_percent(interval=None)  # CPU使用率
            memory = psutil.virtual_memory()  # メモリ情報

            message = String()
            message.data = f"CPU: {cpu_usage}%, Memory: {memory.percent}%"

            self.publisher_.publish(message)
            self.get_logger().info(f"Published: {message.data}")
        except Exception as e:
            self.get_logger().error(f"Error publishing system resources: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ResourceMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f"Error during spin: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

