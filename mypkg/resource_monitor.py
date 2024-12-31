import rclpy
from rclpy.node import Node
import psutil  # システム情報を取得するためのライブラリ
from std_msgs.msg import String

class ResourceMonitor(Node):
    def __init__(self):
        super().__init__('resource_monitor')
        self.publisher_ = self.create_publisher(String, 'system_resources', 10)
        self.timer = self.create_timer(1.0, self.publish_resource_usage)  # 1秒ごとにデータ送信

    def publish_resource_usage(self):
        # CPUとメモリの使用率を取得
        cpu_usage = psutil.cpu_percent(interval=None)  # CPU使用率
        memory = psutil.virtual_memory()  # メモリ情報

        # メッセージを生成
        message = String()
        message.data = f"CPU: {cpu_usage}%, Memory: {memory.percent}%"

        # トピックにデータをパブリッシュ
        self.publisher_.publish(message)
        self.get_logger().info(f"Published: {message.data}")

def main(args=None):
    rclpy.init(args=args)
    node = ResourceMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

