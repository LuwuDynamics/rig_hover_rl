# Hover 平衡策略训练

先按[仓库首页](../README.md)创建 Python 3.11 虚拟环境。以下命令均在 `training/` 目录运行。训练与评估使用简化碰撞几何的 MuJoCo 模型，电脑无需连接机器人。

## MuJoCo 模型

默认训练使用轻量的 [hover_dyn.xml](models/hover_dyn.xml)。[hover.xml](models/hover.xml) 使用相同的动力学参数，并从同目录的 `meshes/` 加载 26 个 STL 外观零件。可以直接打开查看整机：

```sh
python -m mujoco.viewer --mjcf=models/hover.xml
```

要用带外观的模型训练，在下方任一 `train.py` 命令末尾加上 `--model models/hover.xml`。训练仍在后台采样。

## 评估随附策略

```sh
python evaluate.py --policy weights/actor.json --scenarios stand fwd_back turn --seeds 2 --workers 1
```

`stand` 测原地站立，`fwd_back` 测前进和后退，`turn` 测左右转向。增加 `--seeds` 可比较更多随机条件。若使用可选的 `--plot` 输出曲线，先安装 `matplotlib`。

## 从零训练与续训

先跑一个只有 256 步的短训练，确认 MuJoCo、PyTorch、采样进程和模型保存都正常：

```sh
python train.py --name smoke --steps 256 --workers 1 --envs-per-worker 1 --horizon 128 --minibatch 64 --epochs 1 --threads 1
```

每次训练的权重、配置和日志都在 `runs/<name>/`。短训练用于检查流程，得到的策略可能马上摔倒。从零开始较长训练可以运行 `python train.py --name balance --steps 40000000`。每次用新名称保留之前的结果。

若想从随固件提供的策略继续训练，可加载随附的 PyTorch 检查点：

```sh
python train.py --name finetune --resume weights/hover_balance.pt --steps 1000000
python evaluate.py --policy runs/finetune/actor.json --scenarios stand fwd_back turn --seeds 4 --workers 1
```

`hover_balance.pt` 包含 actor 和 critic，`actor.json` 是同一策略的推理参数。随附部署策略取最终训练阶段末尾 24 个检查点的 actor 参数平均值。从头训练会得到自己的结果。

## 导出到 ESP32

安装支持 C++17 的编译器。导出脚本默认调用 `c++`，可用环境变量 `CXX` 选择编译器。先用随附权重验证整个过程：

```sh
python tools/export_policy.py --actor weights/actor.json --name hover_balance_toy --out exported
cmp exported/hover_policy_weights.h ../main/boards/hover/hover_policy_weights.h
```

脚本用两段仿真生成输入和期望轮指令，编译 C++ 推理并逐步比较结果。输出应显示 `C++ replay OK`，`cmp` 成功时没有输出。名称会写入头文件注释，上述名称与随附头文件相同。

训练出自己的策略后，导出到单独目录；评估和回放通过后，把权重复制到固件，再按首页说明编译：

```sh
python tools/export_policy.py --actor runs/finetune/actor.json --name hover_finetune --out exported/finetune
cp exported/finetune/hover_policy_weights.h ../main/boards/hover/
```

## Python 与固件之间的接口

策略采用 9 → 32 → 32 → 2 的前馈网络。输入依次为俯仰偏差、俯仰角速度、偏航角速度、滤波轮速、目标线速度、目标转速，以及速度误差、转速误差和位置误差的积分。两个输出分别控制两轮的共同分量和差动分量。板端按左轮「共同量减差动量」、右轮「共同量加差动量」混合，再做限幅和电机方向映射。

训练时，critic 可读取额外的仿真状态；部署时只运行 actor。[观测与奖励](hover_sim/env.py) · [网络结构](hover_sim/policy.py) · [固件推理](../main/boards/hover/hover_policy.h)
