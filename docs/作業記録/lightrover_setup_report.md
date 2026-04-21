# ライトローバー セットアップ作業報告書
**作業日：2026年4月21日**

---

## 概要

Raspberry Pi 4搭載のライトローバー（ヴィストン株式会社製）に対して、
Ubuntu 22.04 LTSのインストールからROS2 Humbleによるモーター制御起動まで、
一通りの環境構築を行った。

---

## 1. OSのインストール

### 作業内容
- Raspberry Pi Imagerを使用してmicroSDカードに**Ubuntu 22.04 LTS（64bit デスクトップ版）**を書き込み
- RPiにモニター・キーボードを接続して初回セットアップを実施

### 発生した問題と解決策
| 問題 | 原因 | 解決策 |
|------|------|--------|
| セットアップ中に「applying changes」で停止 | ネットワークなしでアップデートを試みた | Cancelボタンで中断 |
| `apt install`時にロックエラー | 別プロセスがaptを使用中 | `sudo rm /var/lib/dpkg/lock-frontend` でロック解除後、`sudo dpkg --configure -a` で修復 |

---

## 2. SSH接続の設定

### 作業内容
RPiにSSHサーバーをインストールし、ラップトップ（WSL）からリモート接続できるようにした。

```bash
sudo apt install openssh-server -y
sudo systemctl enable ssh
sudo systemctl start ssh
ip a  # IPアドレス確認
```

### 接続方法
```bash
ssh mukougawakouhei@150.89.170.112  # LANケーブル経由
```

### 発生した問題と解決策
| 問題 | 原因 | 解決策 |
|------|------|--------|
| `waiting for cache lock`エラー | aptが別プロセスに占有されていた | `sudo kill [PID]`で強制終了 |

---

## 3. 便利なコマンドの設定（WSL側）

### .bashrcにエイリアスを追加

```bash
alias connect='ssh mukougawakouhei@192.168.4.1'    # ホットスポット（メイン）
alias connectLAN='ssh mukougawakouhei@150.89.170.112'  # LANケーブル
alias connectWiFi='ssh mukougawakouhei@172.20.10.11'   # テザリング
```

反映：
```bash
source ~/.bashrc
```

---

## 4. ホスト名の変更（RPi側）

```bash
sudo hostnamectl set-hostname rpi
sudo reboot
```

これにより、ターミナルのプロンプトが短くなった：
```
mukougawakouhei@rpi:~$
```

---

## 5. ROS2 Humbleのインストール

### インストール手順

```bash
# リポジトリの追加
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) \
  signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu \
  $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list

# インストール
sudo apt update
sudo apt install -y ros-humble-desktop
```

### 自動読み込みの設定

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 6. Pythonによるモーター制御（動作確認）

### I2Cアドレスの確認

```bash
sudo apt install i2c-tools -y
sudo i2cdetect -y 1
```

結果：アドレス `0x10` にモータードライバー（VS-WRC201）を確認

### テストコード（motor_test.py）

```python
import smbus
import time

bus = smbus.SMBus(1)
ADDR = 0x10

def write_reg(reg, data):
    bus.write_i2c_block_data(ADDR, reg, data)

def motor_enable():
    write_reg(0x10, [0x03])

def motor_disable():
    write_reg(0x10, [0x00])

def set_target(pos0, pos1):
    write_reg(0x40, [
        (pos0 >> 24) & 0xFF,
        (pos0 >> 16) & 0xFF,
        (pos0 >> 8) & 0xFF,
        pos0 & 0xFF
    ])
    write_reg(0x44, [
        (pos1 >> 24) & 0xFF,
        (pos1 >> 16) & 0xFF,
        (pos1 >> 8) & 0xFF,
        pos1 & 0xFF
    ])

motor_enable()
set_target(100, -100)  # 前進
time.sleep(2)
set_target(0, 0)
motor_disable()
```

実行：
```bash
sudo python3 ~/motor_test.py
```

**→ ライトローバーの走行を確認！**

---

## 7. Wi-Fi接続とホットスポット設定

### Wi-Fi（テザリング）への接続

```bash
sudo nmcli radio wifi on
sudo nmcli connection add type wifi ifname wlan0 con-name "16pro" \
  ssid "16pro" wifi-sec.key-mgmt wpa-psk wifi-sec.psk "1234567890"
sudo nmcli connection up "16pro"
```

### ホットスポット（アクセスポイント）の設定

必要なパッケージのインストール：
```bash
sudo apt install -y hostapd dnsmasq
```

hostapd設定（`/etc/hostapd/hostapd.conf`）：
```
interface=wlan0
ssid=LiteRover
hw_mode=g
channel=6
wpa=2
wpa_passphrase=rover1234
wpa_key_mgmt=WPA-PSK
rsn_pairwise=CCMP
```

dnsmasq設定（`/etc/dnsmasq.conf`）：
```
bind-interfaces
interface=wlan0
dhcp-range=192.168.4.2,192.168.4.20,255.255.255.0,24h
dhcp-option=3,192.168.4.1
```

### 起動時の自動設定（systemdサービス）

wlan0に固定IPを設定するサービス（`/etc/systemd/system/wlan0-hotspot-ip.service`）：
```ini
[Unit]
Description=Set static IP for wlan0 hotspot
Before=hostapd.service dnsmasq.service
After=network-pre.target

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/sbin/ip addr add 192.168.4.1/24 dev wlan0
ExecStart=/sbin/ip link set wlan0 up
ExecStop=/sbin/ip addr del 192.168.4.1/24 dev wlan0

[Install]
WantedBy=multi-user.target
```

NetworkManagerがwlan0を管理しないよう設定：
```bash
sudo tee /etc/NetworkManager/conf.d/wlan0-unmanaged.conf << 'EOF'
[keyfile]
unmanaged-devices=interface-name:wlan0
EOF
```

サービスの有効化：
```bash
sudo systemctl daemon-reload
sudo systemctl enable wlan0-hotspot-ip.service
sudo systemctl enable hostapd
sudo systemctl enable dnsmasq
```

**→ RPi起動時に自動でLiteRoverのSSIDが発信されるようになった！**

### 接続方法
```
SSID: LiteRover
パスワード: rover1234
SSH: ssh mukougawakouhei@192.168.4.1
```

---

## 8. 公開鍵認証の設定

毎回パスワードを入力しなくて済むように設定した。

```bash
# ラップトップ側
ssh-keygen -t ed25519
ssh-copy-id mukougawakouhei@192.168.4.1
```

**→ connectコマンドだけで即接続できるようになった！**

---

## 9. ROS2パッケージのインストールとビルド

### ライトローバー用ROS2パッケージの取得

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/vstoneofficial/lightrover_ros2.git
```

### ビルド

```bash
sudo apt install python3-colcon-common-extensions -y
cd ~/ros2_ws
colcon build
```

### ワークスペースの自動読み込み

```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 10. ROS2でライトローバーを起動

### 必要な追加設定

```bash
# i2cとdialoutグループへの追加
sudo usermod -a -G i2c mukougawakouhei
sudo usermod -a -G dialout mukougawakouhei

# tf_transformationsのインストール
sudo apt install ros-humble-tf-transformations -y
```

### 起動コマンド

```bash
ros2 launch lightrover_ros pos_joycon.launch.py
```

### 起動確認

```
i2c_controller → Service is start        ✅
rover_gamepad  → Game pad node start     ✅
pos_controller → Start POS Controll      ✅
odom_manager   → Start odom manager      ✅
```

**→ ROS2でライトローバーの全ノードが正常起動！**

---

## 今後の課題

- [ ] キーボードでのライトローバー操作
- [ ] LiDARの動作確認
- [ ] SLAMの実行
- [ ] スパース最適制御の実装

---

## ネットワーク構成まとめ

| 接続方法 | IPアドレス | エイリアス |
|---------|-----------|-----------|
| ホットスポット（メイン） | 192.168.4.1 | connect |
| LANケーブル | 150.89.170.112 | connectLAN |
| テザリング | 172.20.10.11 | connectWiFi |
