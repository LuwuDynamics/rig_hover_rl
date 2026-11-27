# RIG Hover RL

ESP32-S3 firmware and a MuJoCo/PyTorch training environment for the RIG Hover two-wheel balancing robot, based on firmware 3.8.4. The repository includes the deployed policy as JSON, a PyTorch checkpoint for fine-tuning, and the C++ weights used by the firmware. [中文](README.md)

[Changelog](CHANGELOG.md)

## Run the policy on a computer

```sh
cd training
python3.11 -m venv .venv
. .venv/bin/activate
python -m pip install -r requirements.txt
python evaluate.py --policy weights/actor.json --scenarios stand fwd_back turn --seeds 2 --workers 1
```

The three scenarios evaluate standing, forward/backward motion and turning. For a short PPO training run, weight export and C++ replay, follow the commands in the [training guide](training/README.md). The shipped actor has nine inputs and two outputs, common and differential wheel commands.

## Flash prebuilt firmware

Connect the RIG Hover with a USB data cable and open the [XGO firmware flasher](https://tools.xgorobot.com/) in your browser. Select RIG Hover, choose the Chinese or international firmware, and follow the on-screen instructions.

## Build the firmware from source

From the repository root, use a shell with ESP-IDF 5.5.3 and its ESP32-S3 tools activated. Asset generation also requires NumPy and Pillow in the IDF Python environment.

```sh
python -m pip install numpy Pillow
./tools/firmware.sh cn build
./tools/firmware.sh gb build
```

Use `cn` for the Chinese edition or `gb` for the international edition. To flash a Hover, replace the port with your device's serial port and run `./tools/firmware.sh gb -p /dev/ttyACM0 flash monitor`. The firmware target expects 16 MB flash and PSRAM. This source edition omits the internal web tuning panel and experiment logging.

Project-owned firmware code, training code and trained policy weights are licensed under [Apache-2.0](LICENSE). See [NOTICE](NOTICE) for third-party materials.
