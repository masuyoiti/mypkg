# mypkg
- このリポジトリは授業で作成したROS 2のパッケージです。
- このパッケージはユーザーが使用しているUbuntu(Linux)のCPUの使用率,ディスクとネットワークのi/o速度,PC全体のメモリ使用率を表示するものである。
# 目次
- 各ファイルの説明
- リポジトリのクローン方法
- 実行方法

# 各ファイルの説明
- package.xml

    モジュール登録に用いたファイルです。
- setup.py

    スクリプトの登録や```launch/talk_listen.launch.py```でノードを纏める時に使ったファイルです。
- .github/workflow/test.yml

    テストバッジのプログラムです。
- test/test.bash

    ```system_monitor```が正常に動作できているかについてのテストプログラムです。
- launch/talk_listen.launch.py

    ```talker.py```と```listener.py```の２つのノードを同時に実行できるノードです。
# リポジトリのクローン方法
以下のコマンドをターミナル上で入力します。
```
git clone https://github.com/masuyoiti/mypkg.git
```
# 実行方法
以下のコマンドを1つ目のターミナル上で入力します。
```
cd ~/ros2_ws
colcon build
source install/setup.bash
ros2 run mypkg system_monitor
```

次に以下のコマンドを2つ目のターミナル上で入力します。
```
ros2 topic echo /system_resources
```
# 実行例
```
data: 'CPU: 8.4%, Memory: 504.85MB, Disk Read: 2.57 MB/s, Disk Write: 0.00 MB/s, Net Sent: 5.37 Mbps, Net Recv: 5.37 Mbps'
---
data: 'CPU: 0.0%, Memory: 504.71MB, Disk Read: 0.00 MB/s, Disk Write: 0.00 MB/s, Net Sent: 0.30 Mbps, Net Recv: 0.30 Mbps'
---
data: 'CPU: 0.1%, Memory: 504.70MB, Disk Read: 0.00 MB/s, Disk Write: 0.04 MB/s, Net Sent: 0.00 Mbps, Net Recv: 0.00 Mbps'
---
data: 'CPU: 0.2%, Memory: 504.70MB, Disk Read: 0.00 MB/s, Disk Write: 0.00 MB/s, Net Sent: 0.56 Mbps, Net Recv: 0.56 Mbps'
---
data: 'CPU: 0.0%, Memory: 504.70MB, Disk Read: 0.00 MB/s, Disk Write: 0.00 MB/s, Net Sent: 0.28 Mbps, Net Recv: 0.28 Mbps'
```
## 必要なソフトウェア
- ros2
 - 使用バージョン：jazzy
## テスト環境
Ubuntu 24.04 LTS
## 参考資料
[ユーザー情報の取得の方法](https://kamedassou.com/python_os_cpu_disk_infomation/)

[ハードウェア情報の取得の方法](https://chantastu.hatenablog.com/entry/2023/07/15/114657#2-CPU%E6%83%85%E5%A0%B1%E3%81%AE%E5%8F%96%E5%BE%97)

[bpsの意味と、通信速度や転送時間の計算方法](https://mathwords.net/bps)

[ネットワーク　伝送速度の考え方](https://www.sumappu.com/post-410/#)

[ネットワークの回線速度と帯域と伝送効率、伝送時間の関係](https://itmanabi.com/network-speed/)
## ライセンス
- このソフトウェアパッケージは、 3条項BSDライセンスの下、 再頒布および使用が許可されます。


© 2024 Youichi Masuyama
