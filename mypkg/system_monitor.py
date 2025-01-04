# SPDX-FileCopyrightText: 2024 Youichi Masuyama <yaiti0212@gmail.com>
# SPDX-License-Identifier: BSD-3-Clause

from rclpy.node import Node
import psutil
import rclpy
from std_msgs.msg import String
import time
from colorama import Fore, Style

class ResourceMonitor(Node):
    def __init__(self):
        super().__init__('resource_monitor')
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
            disk_read = (current_disk_io.read_bytes - self.prev_disk_io.read_bytes) / (1024 * 1024 * elapsed_time)
            disk_write = (current_disk_io.write_bytes - self.prev_disk_io.write_bytes) / (1024 * 1024 * elapsed_time)

            # ネットワーク速度の計算
            current_net_io = psutil.net_io_counters()
            net_sent = (current_net_io.bytes_sent - self.prev_net_io.bytes_sent) * 8 / (1024 * 1024 * elapsed_time)
            net_recv = (current_net_io.bytes_recv - self.prev_net_io.bytes_recv) * 8 / (1024 * 1024 * elapsed_time)

            # 色付きのメッセージ
            colored_message = (
                f"{Fore.RED}CPU: {cpu_usage}%{Style.RESET_ALL}, "
                f"{Fore.BLUE}Memory: {memory.percent}%{Style.RESET_ALL}, "
                f"{Fore.GREEN}Disk Read: {disk_read:.2f} MB/s{Style.RESET_ALL}, "
                f"{Fore.YELLOW}Disk Write: {disk_write:.2f} MB/s{Style.RESET_ALL}, "
                f"{Fore.MAGENTA}Net Sent: {net_sent:.2f} Mbps{Style.RESET_ALL}, "
                f"{Fore.CYAN}Net Recv: {net_recv:.2f} Mbps{Style.RESET_ALL}"
            )
            print(f"Publishing: {colored_message}")

            # メッセージを発行
            message = String()
            message.data = colored_message  # 色付きでないデータを使用する場合は変換が必要
            if rclpy.ok():
                self.publisher_.publish(message)

            # 状態を更新
            self.prev_disk_io = current_disk_io
            self.prev_net_io = current_net_io
            self.prev_time = current_time

        except Exception as e:
            if rclpy.ok():
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
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()

