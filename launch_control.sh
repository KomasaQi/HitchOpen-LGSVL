
#!/bin/bash

# 进入目标目录（只需一次 cd）
cd ~/catkinws/HitchOpen-LGSVL || exit 1

# 第二条命令：后台运行 ros2 launch（执行前等待 2 秒）
ros2 launch autonomy_launch misc.launch.py &

# 再次等待 2 秒（给第二条命令启动时间）
sleep 1

# 第三条命令：前台运行 ros2 launch（不再后台，保持原逻辑）
ros2 launch simple_racing simple_racing.launch.py params_file:=src/launch/simple_racing/params/simple_racing.yml