# Changelog

## 3.8.5 · 2026-11-27

### 中文

- 设备启动时，以及联网后唤醒或开始聊天时，会检查绑定状态。确认解绑后显示绑定二维码并保留已保存的 Wi-Fi。修正已绑定设备因 OTA 占位地址再次显示绑定页的问题。
- 缩小 MCP 工具清单，为分页预留空间。串口日志会记录每页的工具数量、字节数和下一页游标，方便排查语音控制问题。
- 轮毂测试工具 self.robot.motor11 和 self.robot.motor21 合并为 self.robot.motor_test，通过 id=11 或 id=21 选择电机。

### English

- Hover checks its binding status at startup and when a voice session starts while online. After unbinding, it shows the binding QR code and keeps the saved Wi-Fi network. A placeholder OTA URL no longer sends an already-bound device back to the binding screen.
- The MCP tool list is smaller and leaves space for pagination. Serial logs report the tool count, byte count and next cursor for each page to help diagnose voice control issues.
- The wheel test tools self.robot.motor11 and self.robot.motor21 have been merged into self.robot.motor_test. Pass id=11 or id=21 to select the motor.

## 3.8.4 · 2026-11-27

### 中文

- 配网二维码显示时，轻触头部只摇头，动作结束后二维码仍可扫描。配网期间的单击和双击均不播放触摸音效。
- IMU 校准完成或读数中断后恢复时，使用当前加速度重新确定机身姿态。连续摆正满 1.5 秒才进入平衡，中断前的时间不计入。
- 站稳提示音现在与 RL 接管过程同步。机身持续稳定时，接管完成即触发音效。如果还在晃动，就等站稳后再响。提示音可以叠加在配网语音上。

### English

- While the Wi-Fi setup QR code is on screen, tapping the head shakes it and keeps the code available to scan throughout the gesture. Single and double taps are silent during setup.
- After IMU calibration or a gap in sensor readings, Hover uses the current accelerometer reading to recover its tilt. It enters balance after 1.5 continuous seconds upright. Time before an interruption does not count.
- Hover now checks stability during the RL controller handover. If the robot stays steady, the chime triggers when handover completes. If it is still moving, the chime waits until it settles. The chime can play over Wi-Fi setup speech.
