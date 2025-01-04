# SPDX-FileCopyrightText: 2024 Youichi Masuyama <yaiti0212@gmail.com>
# SPDX-License-Identifier: BSD-3-Clause

from rclpy.node import Node
import psutil
import rclpy
from std_msgs.msg import String
import time

class ResourcePublisher(Node):
    def __init__(self):
        super().__init__('resource_publisher')
        self.publisher_ = self.create_publisher(String, 'system_resources', 10)
        self.timer = self.create_timer(1.0, self.publish_resource_usage)

        # 初期値の保存
        self.prev_disk_io = psutil.disk_io_counters()
        self.prev_net_io = psutil.net_io_counters()
        self.prev_time = time.time()

    def publish_resource_usage(self):
        try:
            # 現在の時刻
            current_time = time.time()
            elapsed_time = current_time - self.prev_time

            # システムリソース
            cpu_usage = psutil.cpu_percent(interval=None)  # CPU使用率
            memory = psutil.virtual_memory()  # メモリ情報

            # ディスク使用率の計算
            current_disk_io = psutil.disk_io_counters()
            disk_read = (current_disk_io.read_bytes - self.prev_disk_io.read_bytes) / (1024 * 1024 * elapsed_time)  # MB/s
            disk_write = (current_disk_io.write_bytes - self.prev_disk_io.write_bytes) / (1024 * 1024 * elapsed_time)  # MB/s

            # ネットワーク速度の計算
            current_net_io = psutil.net_io_counters()
            net_sent = (current_net_io.bytes_sent - self.prev_net_io.bytes_sent) * 8 / (1024 * 1024 * elapsed_time)  # Mbps
            net_recv = (current_net_io.bytes_recv - self.prev_net_io.bytes_recv) * 8 / (1024 * 1024 * elapsed_time)  # Mbps

            # メッセージ
            message = (
                f"CPU: {cpu_usage:.1f}%, "
                f"Memory: {memory.percent:.1f}%, "
                f"Disk Read: {disk_read:.2f} MB/s, "
                f"Disk Write: {disk_write:.2f} MB/s, "
                f"Net Sent: {net_sent:.2f} Mbps, "
                f"Net Recv: {net_recv:.2f} Mbps"
            )

            # メッセージを発行
            msg = String()
            msg.data = message
            if rclpy.ok():
                self.publisher_.publish(msg)

            # 状態を更新
            self.prev_disk_io = current_disk_io
            self.prev_net_io = current_net_io
            self.prev_time = current_time

        except Exception as e:
            if rclpy.ok():
                self.get_logger().error(f"Error publishing system resources: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ResourcePublisher()
    print("ResourcePublisher node started. Publishing system resources to 'system_resources' topic.")  # パブリッシャ起動時の確認メッセージ
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error during spin: {e}")
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
