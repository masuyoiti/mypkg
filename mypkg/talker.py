import rclpy
from rclpy.node import Node
import psutil  # システム情報を取得するためのライブラリ
from std_msgs.msg import String

class ResourcePublisher(Node):
    def __init__(self):
        super().__init__('resource_publisher')
        self.publisher_ = self.create_publisher(String, 'system_resources', 10)
        self.timer = self.create_timer(1.0, self.publish_resource_usage)  # 1秒ごとにデータ送信

    def publish_resource_usage(self):
        # CPU、メモリ、ディスク、ネットワークの使用状況を取得
        cpu_usage = psutil.cpu_percent(interval=None)  # CPU使用率
        memory = psutil.virtual_memory()  # メモリ情報
        disk = psutil.disk_usage('/')  # ディスク使用状況
        net = psutil.net_io_counters()  # ネットワーク使用状況

        # メッセージを生成
        message = String()
        message.data = (
            f"CPU: {cpu_usage}%, "
            f"Memory: {memory.percent}%, "
            f"Disk: {disk.percent}%, "
            f"Network: Sent={net.bytes_sent / (1024 ** 2):.2f}MB, "
            f"Received={net.bytes_recv / (1024 ** 2):.2f}MB"
        )
        
        # トピックにパブリッシュ
        self.publisher_.publish(message)
        self.get_logger().info(f"Published: {message.data}")

def main(args=None):
    rclpy.init(args=args)
    node = ResourcePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

