# pressure_simple

シリアル経由で圧力要求コマンドを送信し、受信した8ch(1byte/ch)の圧力値をROS 2のトピックに配信する最小構成ノード（ROS 2 Humble）。

- 送信フレーム: `AA C1 [ID] 00 21 55` （IDは `0x0A/0x0B/0x0C` = 10/11/12）
- 受信フレーム: `AA C8 [ID] 00 d1 d2 d3 d4 d5 d6 d7 d8 55` （8chぶん各1byte）
- 公開トピック:
  - `pressure/board/10` (`std_msgs/UInt8MultiArray`, 長さ8)
  - `pressure/board/11` (`std_msgs/UInt8MultiArray`, 長さ8)
  - `pressure/board/12` (`std_msgs/UInt8MultiArray`, 長さ8)

## ビルドと起動

```bash
colcon build --packages-select pressure_simple
source install/setup.bash
ros2 launch pressure_simple pressure_serial.launch.py
```

## 受信確認

```bash
ros2 topic echo /pressure/board/10
ros2 topic echo /pressure/board/11
ros2 topic echo /pressure/board/12
```

## パラメータ

- `serial_port` (string, default: `/dev/ttyUSB0`)
- `baud_rate` (int, default: `2000000`)
- `command_delay_ms` (int, default: `100`)
- `board_ids` (int array, default: `[10, 11, 12]`)

## 注意

- `B2000000` が利用できるLinux環境を想定しています。
- 受信データはそのままの1byte値（0–255）を配信します（スケーリングや平均化なし）。