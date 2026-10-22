# RPi側 `odometry.py` i2c要求の再送パッチ

**このパッチが無いと3代目以降の車体では `/odom` が永久に出ない。**
2026-10-15 に3代目RPi4で原因特定・実機で検証済み。出所は
`docs/作業記録/2026-10-15_3代目セットアップとodom再送パッチ.md`。

**ここに退避している理由**: パッチの実体は RPi の SD カード上にしか無く、
カードが死ぬと失われるため（卒研の実機作業が丸ごと止まる）。

## 対象

RPi 側 `~/ros2_ws/src/lightrover_ros2/lightrover_ros2/odometry.py`（ヴィストン製・ベンダ原版）。
**適用後に `colcon build` が必要。**

## 症状

起動直後に `odom_manager` が `rclpy.spin_once`（`odometry.py:119` 付近）で待ったまま固まり、
`/odom` が永久に出ない。サービスサーバ自体は正常（`ros2 service call /wrc201_i2c` は即答）。

## 原因

`odometry.py` は起動直後に2件の i2c サービス要求を投げるが、**応答を1回でも取りこぼすと
再送しない**。`getEncVal()` の再呼び出しが publish 成功の内側にしかないため、
DDS のディスカバリ完成前に投げた要求の応答が落ちると自力では復帰できない。
3代目の車体では**毎回取りこぼす**。

## 差分

> **注意**: 行番号は未確定（ベンダ原版の版により前後する）。`patch -p1` で当てずに
> 手で編集すること。`import time` が無ければ追加する。

```diff
     odom_manager.getEncVal()
+    last_req = time.time()

     while rclpy.ok():
-        rclpy.spin_once(odom_manager)
+        rclpy.spin_once(odom_manager, timeout_sec=0.1)
         get_val = None
         if odom_manager.enc_a.done() and odom_manager.enc_b.done():
             get_val_a = odom_manager.enc_a.result().read_data
             get_val_b = odom_manager.enc_b.result().read_data
             get_val = calSpeed([get_val_a, get_val_b])

         if get_val is None:
+            # 応答を1回でも取りこぼすと再送が無いため永久に待ち続ける。
+            # 起動直後のDDSディスカバリ競合で実際に発生した（2026-10-15）。
+            if time.time() - last_req > 1.0:
+                odom_manager.get_logger().warn(
+                    'i2c応答が1秒来ないので要求を再送する')
+                odom_manager.getEncVal()
+                last_req = time.time()
             continue
```

さらに **publish 成功後の `getEncVal()` の直後にも** `last_req = time.time()` を足す。

## 検証（合格基準）

- `/odom` が **約30Hz** で流れる（3代目実測 31.46Hz。7/15実機bagの基準は 29.94Hz・中央間隔32.7ms）
- `odom_manager.log` に再送の警告が**起動ごとに1回**出てから復帰する

## やってはいけない中間版

待機中に `time.sleep(0.025)` を入れると **odom が 6.2Hz まで落ちる**（1周期160ms）。
odom の報告遅れはむだ時間135msの構成要素なので致命的。
`spin_once(timeout_sec=0.1)` がブロックするので sleep は不要。

## 関連

同じファイルの `ROVER_D` は**車体固有**。新しい車体では `spin_test` で再較正すること
（3代目は `0.1514 → 0.1525`・**検証旋回は未実施**）。`pos_controller.py` 側も同じ値にする。
