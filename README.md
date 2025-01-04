[![test](https://github.com/masuyoiti/mypkg/actions/workflows/test.yml/badge.svg)](https://github.com/masuyoiti/mypkg/actions/workflows/test.yml)

# mypkg

- このリポジトリは授業で作成したROS 2のパッケージです。
- このパッケージは、ホストマシンのシステムリソースを監視する ROS 2 ノードを提供します。監視するリソースには、CPU使用率、アクティブなメモリ使用量（プロセスによる物理メモリ使用を概算したもの）、ディスクI/O、ネットワークI/Oが含まれます。このノードは、監視した情報を `system_resources` というROS 2トピックに発行します。

## 機能

- **CPU使用率**: CPU使用率（%）
- **アクティブメモリ使用量**: アクティブなプロセスによって使用されている物理メモリ（MB単位）
- **ディスクI/O**: ディスクの読み取り・書き込み速度（MB/s）
- **ネットワークI/O**: ネットワーク送信・受信速度（Mbps）


## インストール方法

1. **必要条件**:
   - ROS 2(jazzy、humble)をインストールしてください。
   - システム監視のために `psutil` をインストールしてください。
     ```bash
     pip install psutil
     ```

2. **パッケージをクローン**:
   - このリポジトリをROS 2ワークスペースの `src` フォルダにクローンします。
     ```bash
     cd ~/ros2_ws/src
     git clone https://github.com/masuyoiti/mypkg.git
     ```

3. **パッケージをビルド**:
     ```bash
     cd ~/ros2_ws
     colcon build
     ```

4. **ワークスペースをソース**
     ```bash
     source ~/ros2_ws/install/setup.bash
     ```

## 使用方法

1. **ノードを起動**
   ROS 2ワークスペースをソースした後、以下のコマンドで`ResourcePublisher`ノードを起動します。
     ```bash
     ros2 run mypkg system_monitor
     ```

2. **サブスクライブするトピック**
   ノードは、システムリソース情報を `system_resources`というトピックに発行します。このトピックを`ros2 topic echo`コマンドで確認できます。
     ```bash
     ros2 topic echo /system_resources
     ```

## 実行例
```
data: 'CPU: 8.4%, Memory: 504.85MB, Disk Read: 2.57 MB/s, Disk Write: 0.00 MB/s, Net Sent: 5.37 Mbps, Net Recv: 5.37 Mbps'
---
data: 'CPU: 0.0%, Memory: 504.71MB, Disk Read: 0.00 MB/s, Disk Write: 0.00 MB/s, Net Sent: 0.30 Mbps, Net Recv: 0.30 Mbps'
---
data: 'CPU: 0.1%, Memory: 504.70MB, Disk Read: 0.00 MB/s, Disk Write: 0.04 MB/s, Net Sent: 0.00 Mbps, Net Recv: 0.00 Mbps'
---
```

## ノードの概要
- ノード名：`resource_publisher`
- トピック: `/system_resources`
    - 型: `std_msgs/msg/String`

## コード構成
- `system_monitor.py`: システムリソースを監視し、定期的に情報をROS 2トピックに発行するメインのPythonスクリプト。`psutil`ライブラリを使用してリソース情報を取得し、1秒ごとに結果を発行します。
- `requirements.txt`: パッケージのPython依存関係（例:`psutil`）をリストします。

## 依存関係
- ROS 2: ROS 2 環境でノードを実行するために必要です。
- psutil: システムとプロセスの情報を取得するためのPythonライブラリ。CPU、メモリ、ディスクI/O、ネットワークI/Oなどの情報を提供します。

## 注意事項
- ネットワーク依存: ネットワーク環境によっては、ネットワーク速度の値が一時的に不正確になる可能性があります。
- 高負荷時のパフォーマンス: システム負荷が極端に高い場合、リソース取得に遅延が発生する可能性があります。 

## テスト環境
- Ubuntu 22.04 LTS
    - ROS 2 Humble(GitHub Actions)
- Ubuntu 24.04 LTS
    - ROS 2 jazzy(開発環境)
## 参考資料
[ユーザー情報の取得の方法](https://kamedassou.com/python_os_cpu_disk_infomation/)

[ハードウェア情報の取得の方法](https://chantastu.hatenablog.com/entry/2023/07/15/114657#2-CPU%E6%83%85%E5%A0%B1%E3%81%AE%E5%8F%96%E5%BE%97)

[bpsの意味と、通信速度や転送時間の計算方法](https://mathwords.net/bps)

[ネットワーク　伝送速度の考え方](https://www.sumappu.com/post-410/#)

[ネットワークの回線速度と帯域と伝送効率、伝送時間の関係](https://itmanabi.com/network-speed/)
## ライセンス
- このソフトウェアパッケージは、 3条項BSDライセンスの下、 再頒布および使用が許可されます。
- このパッケージのコードの一部は、下記のスライド（CC-BY-SA 4.0 by Ryuichi Ueda）のものを本人の許可を得て自身の著作としたものです。
    - [ryuichiueda/slides_marp/tree/master/robosys_2024](https://github.com/ryuichiueda/slides_marp/tree/master/robosys2024)

詳細は LICENSE ファイルを参照してください。

© 2025 Youichi Masuyama
