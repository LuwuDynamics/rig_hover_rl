# Changelog

## 3.8.4 · 2026-11-27

### 中文

- 配网二维码显示时，轻触头部只摇头，动作结束后二维码仍可扫描。配网期间的单击和双击均不播放触摸音效。
- IMU 校准完成或读数中断后恢复时，使用当前加速度重新确定机身姿态。连续摆正满 1.5 秒才进入平衡，中断前的时间不计入。
- 站稳提示音现在与 RL 接管过程同步。机身持续稳定时，接管完成即触发音效。如果还在晃动，就等站稳后再响。提示音可以叠加在配网语音上。

### English

- While the Wi-Fi setup QR code is on screen, tapping the head shakes it and keeps the code available to scan throughout the gesture. Single and double taps are silent during setup.
- After IMU calibration or a gap in sensor readings, Hover uses the current accelerometer reading to recover its tilt. It enters balance after 1.5 continuous seconds upright. Time before an interruption does not count.
- Hover now checks stability during the RL controller handover. If the robot stays steady, the chime triggers when handover completes. If it is still moving, the chime waits until it settles. The chime can play over Wi-Fi setup speech.
