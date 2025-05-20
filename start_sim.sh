#!/bin/bash

# 进入目标目录（只需一次 cd）
cd ~/catkinws/HitchOpen-LGSVL || exit 1

# 第一条命令：后台运行 python 脚本
python3 src/launch/svl_launch/scripts/launch_ossdc.py --env svl.env 
