# leptrino_force_torque

## 概要 (Overview)
株式会社レプトリノ (Leptrino) 製の6軸力覚センサ (6-Axis Force/Torque Sensor) をROS2で使用するためのROSパッケージです。ROS2コンポーネント化に対応し、component containerからの起動が可能です。

-   **センサ製品ページ**: http://www.leptrino.co.jp/P_CFS.html

## 動作確認済みハードウェア (Tested Hardware)
-   **センサモデル**: PFS055YA251U6

## セットアップ (Setup)

### 依存関係 (Dependencies)
-   ROS2 Humble/Iron/Rolling
-   rclcpp
-   rclcpp_components
-   geometry_msgs
-   Leptrino センサドライバライブラリ (`leptrino` ディレクトリにソースコードが含まれています)

### ビルド (Build)
```bash
# ワークスペースルートで
colcon build --packages-select leptrino_force_torque
source install/setup.bash
```

## 実行方法 (Usage)

### 1. 従来のノードとして実行
```bash
ros2 launch leptrino_force_torque leptrino.launch.py
```

### 2. コンポーネントとして実行（推奨）
```bash
ros2 launch leptrino_force_torque leptrino_component.launch.py
```

### 3. 手動でのコンポーネント起動
```bash
# component containerを起動
ros2 run rclcpp_components component_container --ros-args -r __node:=leptrino_container

# 別のターミナルでコンポーネントをロード
ros2 component load /leptrino_container leptrino_force_torque LeptrinoNode
```

### Launch引数
| 引数名 | 型 | 説明 | デフォルト値 |
|:---|:---|:---|:---|
| `comport` | `string` | センサが接続されているシリアルポート | `/dev/ttyACM0` |
| `sampling_rate` | `double` | 発行レート (Hz) | `100.0` |
| `frame_id` | `string` | センサのフレームID | `leptrino_link` |
| `container_name` | `string` | コンテナ名（コンポーネント版のみ） | `leptrino_container` |

### 使用例
```bash
# ポート指定してコンポーネント起動
ros2 launch leptrino_force_torque leptrino_component.launch.py comport:=/dev/ttyACM0

# 高周波数でのデータ取得
ros2 launch leptrino_force_torque leptrino_component.launch.py sampling_rate:=1200.0
```

### COMポートの指定について
USBデバイスは接続するポートや順番によって `/dev/ttyACM0`, `/dev/ttyACM1` のようにデバイス名が変わってしまうことがあります。これを避けるため、各デバイス固有のIDで指定できる `/dev/serial/by-id/` パスを使用することを推奨します。

以下のコマンドで接続されているデバイスのIDを確認できます：
```bash
ls -l /dev/serial/by-id/
```

## ノード仕様 (Node Specification)

### `leptrino_force_torque_node`
センサからデータを読み取り、ROS2トピックとして発行するメインノードです。

#### 発行するトピック (Published Topics)
-   `~/wrench` (`geometry_msgs/WrenchStamped`)
    -   センサから取得した力・トルクデータ

#### パラメータ (Parameters)
| パラメータ名 | 型 | 説明 | デフォルト値 |
|:---|:---|:---|:---|
| `com_port` | `string` | センサが接続されているシリアルポート | `/dev/ttyACM0` |
| `rate` | `double` | `wrench` トピックを公開するレート (Hz) | `1200.0` |
| `frame_id` | `string` | 発行される `WrenchStamped` メッセージのframe_id | `leptrino_link` |

#### 動作仕様
- センサからは最高速（約1200Hz）でデータを読み取り続けます
- `rate`パラメータで指定された周期で最新のデータをROSトピックに公開します
- マルチスレッド処理でセンサ読み取りとROS通信を分離し、安定したパフォーマンスを実現

## 利用可能なファイル (Available Files)
```
launch/
├── leptrino.launch.py           # 従来のノード起動用
└── leptrino_component.launch.py # コンポーネント起動用（推奨）
```

## トラブルシューティング

### シリアルポート権限エラー
```bash
sudo usermod -a -G dialout $USER
# 再ログインが必要
```

### センサが見つからない場合
```bash
# USB接続の確認
lsusb
dmesg | grep tty

# シリアルポートの確認
ls /dev/ttyACM*
```

## ライセンス (License)
このソフトウェアはBSDライセンスの下で公開されています。詳細はソースコードのヘッダを参照してください。

