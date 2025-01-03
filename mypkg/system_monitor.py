from rclpy.node import Node
import psutil
import rclpy
from std_msgs.msg import String
import time


class ResourceMonitor(Node):
    def __init__(self):
        super().__init__('resource_monitor')
        self.publisher_ = self.create_publisher(String, 'system_resources', 10)
        self.timer = self.create_timer(1.0, self.publish_resource_usage)
        
        # 前回のディスクおよびネットワークの値を保持するための変数
        self.prev_disk_io = psutil.disk_io_counters()
        self.prev_net_io = psutil.net_io_counters()
        self.prev_time = time.time()

    def publish_resource_usage(self):
        try:
            # 現在の時刻を取得
            current_time = time.time()
            elapsed_time = current_time - self.prev_time

            # システムリソースを取得
            cpu_usage = psutil.cpu_percent(interval=None)  # CPU使用率
            memory = psutil.virtual_memory()  # メモリ情報

            # ディスク使用率の計算
            current_disk_io = psutil.disk_io_counters()
            disk_read = (current_disk_io.read_bytes - self.prev_disk_io.read_bytes) / (1024 * 1024 * elapsed_time)
            disk_write = (current_disk_io.write_bytes - self.prev_disk_io.write_bytes) / (1024 * 1024 * elapsed_time)

            # ネットワーク使用率の計算
            current_net_io = psutil.net_io_counters()
            net_sent = (current_net_io.bytes_sent - self.prev_net_io.bytes_sent) * 8 / (1024 * 1024 * elapsed_time)
            net_recv = (current_net_io.bytes_recv - self.prev_net_io.bytes_recv) * 8 / (1024 * 1024 * elapsed_time)

            # 結果をメッセージに追加
            message = String()
            message.data = (
                f"CPU: {cpu_usage}%, Memory: {memory.percent}%, "
                f"Disk Read: {disk_read:.2f} MB/s, Disk Write: {disk_write:.2f} MB/s, "
                f"Net Sent: {net_sent:.2f} Mbps, Net Recv: {net_recv:.2f} Mbps"
            )

            # コンソールに出力
            print(f"Publishing: {message.data}")

            # メッセージをトピックに発行
            if rclpy.ok():
                self.publisher_.publish(message)

            # 現在のデータを保存
            self.prev_disk_io = current_disk_io
            self.prev_net_io = current_net_io
            self.prev_time = current_time

        except Exception as e:
            if rclpy.ok():  # エラーをログに記録
                self.get_logger().error(f"Error publishing system resources: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ResourceMonitor()
    print("ResourceMonitor node started.")  # ノード起動時の確認メッセージ
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error during spin: {e}")
    finally:
        if rclpy.ok():  # シャットダウンがまだ呼び出されていない場合のみ実行
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()

