# leptrino_force_torque

## 概要 (Overview)
株式会社レプトリノ (Leptrino) 製の6軸力覚センサ (6-Axis Force/Torque Sensor) をROS1で使用するためのROSパッケージです。

-   **センサ製品ページ**: http://www.leptrino.co.jp/P_CFS.html

## 動作確認済みハードウェア (Tested Hardware)
-   **センサモデル**: PFS055YA251U6

## セットアップ (Setup)

### 依存関係 (Dependencies)
-   ROS1 (Noetic 推奨)
-   Leptrino センサドライバライブラリ (`leptrino` ディレクトリにソースコードが含まれている必要があります)

## 実行方法 (Usage)
`roslaunch`を使用してセンサノードを起動します。

```bash
roslaunch leptrino_force_torque leptrino.launch
```

### launchファイル例 (`leptrino.launch`)
```xml
<launch>
    <arg name="comport" default="/dev/ttyACM0" />

    <arg name="sampling_rate" default="1200" />

    <arg name="frame_id" default="leptrino" />

    <node name="leptrino" pkg="leptrino_force_torque" type="leptrino_force_torque"
        output="screen">
        <param name="com_port" value="$(arg comport)" />
        <param name="frame_id" value="$(arg frame_id)" />
        <param name="rate" value="$(arg sampling_rate)" />
    </node>
</launch>
```

### COMポートの指定について
USBデバイスは接続するポートや順番によって `/dev/ttyACM0`, `/dev/ttyACM1` のようにデバイス名が変わってしまうことがあります。これを避けるため、各デバイス固有のIDで指定できる `/dev/serial/by-id/` パスを使用することを推奨します。

以下のコマンドで接続されているデバイスのIDを確認できます。
```bash
ls -l /dev/serial/by-id/
```

## ノード (Nodes)

### `leptrino_force_torque`
センサからデータを読み取り、ROSトピックとして発行するメインノードです。

#### 発行するトピック (Published Topics)
-   `~wrench` (`geometry_msgs/WrenchStamped`)
    -   センサから取得した力のデータ。デフォルトのトピック名は `/leptrino/wrench` です。

#### パラメータ (Parameters)
| パラメータ名 | 型 | 説明 | デフォルト値 |
|:---|:---|:---|:---|
| `~com_port` | `string` | センサが接続されているシリアルポート。 | `/dev/ttyACM0` |
| `~rate` | `int` | `wrench` トピックを公開するレート (Hz)。ノードはバックグラウンドでセンサから最高速(約1200Hz)でデータを読み取り続け、`ros::Timer`がこのパラメータで指定された周期で最新のデータを正確に公開します。 | `1200` |
| `~frame_id` | `string` | 発行される `WrenchStamped` メッセージのヘッダに書き込まれる `frame_id`。 | `leptrino` |

## ライセンス (License)
このソフトウェアはBSDライセンスの下で公開されています。詳細はソースコードのヘッダを参照してください。

