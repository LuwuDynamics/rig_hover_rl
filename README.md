# RIG Hover RL

RIG Hover 是一台双轮自平衡机器人。本仓库基于 3.8.3 固件，包含 ESP32-S3 源码、MuJoCo 动力学模型、PPO 训练代码和机器人使用的策略权重。在电脑上就能评估策略或开始训练。[English](README_EN.md)

## 先在电脑上跑起来

使用 Python 3.11，在仓库根目录执行以下命令。评估默认在 CPU 上运行。

```sh
cd training
python3.11 -m venv .venv
. .venv/bin/activate
python -m pip install -r requirements.txt
python evaluate.py --policy weights/actor.json --scenarios stand fwd_back turn --seeds 2 --workers 1
```

输出按场景列出完成次数、姿态和速度误差。想训练自己的模型或验证板端推理，请看[训练与权重导出](training/README.md)。

## 烧录预编译固件

用 USB 数据线连接 RIG Hover，在浏览器中打开 [XGO 固件烧录工具](https://tools.xgorobot.com/)，选择 RIG Hover 和对应的国内版或国际版固件，按页面提示烧录。

## 从源码编译与烧录

硬件目标为带 16 MB Flash 和 PSRAM 的 RIG Hover（ESP32-S3）。安装 ESP-IDF 5.5.3 的 ESP32-S3 工具链，在已加载该版本 `export.sh` 的终端中，从仓库根目录运行：

```sh
python -m pip install numpy Pillow
./tools/firmware.sh cn build
./tools/firmware.sh gb build
```

`cn` 选择国内服务和中文资源，`gb` 选择海外服务和英文资源。两版的构建目录分别是 `build-cn/` 和 `build-gb/`，首次编译会按 `dependencies.lock` 安装组件。下面以国内版为例，将串口名换成电脑上实际的设备：

```sh
./tools/firmware.sh cn -p /dev/ttyACM0 flash monitor
```

macOS 的串口通常是 `/dev/cu.usbmodem…`，退出监视器按 `Ctrl+]`。Flash 目标包含程序、分区表、引导程序和匹配的表情与唤醒资源。标定按屏幕提示完成。

## 代码入口

| 想看什么 | 从哪里开始 |
| --- | --- |
| MuJoCo 里的机器人和训练环境 | [动力学模型](training/models/hover_dyn.xml) · [环境、观测和奖励](training/hover_sim/env.py) |
| 训练策略或继续训练 | [PPO 训练器](training/train.py) · [已训练权重](training/weights/) |
| 板端如何算出左右轮指令 | [策略推理](main/boards/hover/hover_policy.h) · [权重头文件](main/boards/hover/hover_policy_weights.h) |
| 策略怎样接入实际控制循环 | [控制器](main/boards/hover/rl_controller.h) · [电机、状态和保护](main/boards/hover/robot.cc) |
| 标定、轻触、动作与倾听 | [Hover 板级逻辑](main/boards/hover/hover_board.cc) · [动作播放器](main/boards/hover/gesture_player.h) |

公开源码沿用 3.8.2 的平衡策略，在这个版本中精简了内部网页调参和试验采样工具。

## 许可

自有固件代码、训练代码和训练策略权重采用 [Apache License 2.0](LICENSE)。第三方组件和随附资源的声明见 [NOTICE](NOTICE)。
