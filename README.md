# mypkg
- このリポジトリは授業で作成したROS2のパッケージです。

# 目次
- 各ファイルの説明
- リポジトリのクローン方法
- 実行方法

# 各ファイルの説明
- package.xml

    モジュール登録に用いたファイルです。
- setup.py

    スクリプトの登録や```launch/talk_listen.launch.py```でノードを纏める時に使ったファイルです.
- .github/workflow/test.yml

    テストバッジのプログラムです。
- test/test.bash

    ```talker.py```と```listener.py```が正常に通信できているかについてのテストプログラムです。
- launch/talk_listen.launch.py

    ```talker.py```と```listener.py```の２つのノードを同時に実行できるノードです。
    また、実行するには以下のコマンドを実行してください。終了には```Ctrl+C```を入力してください。
# リポジトリのクローン方法
以下のコマンドをターミナル上で入力します。
```

```
## 実行方法
- 実行例
```

```
- 実行結果
```

```
## 実行可能ソフトウェア
- Python
 - テスト済みバージョン：
## テスト環境
Ubuntu 22.04 LTS
## 参考資料
[ユーザー情報の取得の方法](https://kamedassou.com/python_os_cpu_disk_infomation/)

[ハードウェア情報の取得の方法](https://chantastu.hatenablog.com/entry/2023/07/15/114657#2-CPU%E6%83%85%E5%A0%B1%E3%81%AE%E5%8F%96%E5%BE%97)

[通信速度の計算](https://mathwords.net/bps)
## ライセンス
- このソフトウェアパッケージは、 3条項BSDライセンスの下、 再頒布および使用が許可されます。


© 2024 Youichi Masuyama
