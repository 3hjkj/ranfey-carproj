#!/bin/bash
# 激活 Conda 环境
source /home/nvidia/zhitai/conda3/etc/profile.d/conda.sh
conda activate rosyolo

# 临时移除 Conda 的 libstdc++，避免干扰 ROS 2 原生 C++ 依赖
export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libstdc++.so.6

# 调用 main.py（建议 shebang 行写 #!/usr/bin/env python）
exec python3 $(ros2 pkg prefix yolo)/lib/yolo/main.py "$@"
