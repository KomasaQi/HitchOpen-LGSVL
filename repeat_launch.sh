#!/bin/bash

if [ $# -ne 1 ]; then
    echo "用法: $0 <循环次数>"
    exit 1
fi

N=$1
TARGET_NODE="/global_planning"  # 根据实际情况修改节点名称

for ((i=1; i<=N; i++)); do
    echo "-------------------------"
    echo "第 $i 次运行 | 剩余次数: $((N-i))"
    
    # 启动launch并记录PID
    ros2 launch simple_racing simple_racing.launch.py params_file:=src/launch/simple_racing/params/simple_racing.yml &
    LAUNCH_PID=$!
    
    # 节点监控循环
    while true; do
        # 检查launch进程是否仍在运行
        if ! kill -0 $LAUNCH_PID 2>/dev/null; then
            echo "ros2 launch进程已终止"
            break
        fi
        
        # 检查目标节点是否存在
        if ! ros2 node list | grep -q "$TARGET_NODE"; then
            echo "检测到节点崩溃，终止进程..."
            kill -INT $LAUNCH_PID  # 发送Ctrl+C等效信号
            wait $LAUNCH_PID 2>/dev/null
            break
        fi
        
        sleep 0.2  # 检查间隔
    done
    
    # 等待进程完全退出
    wait $LAUNCH_PID 2>/dev/null
    
    # 间隔时间（可调整）
    if [ $i -ne $N ]; then
        echo "准备重启，等待3秒..."
        sleep 0.2
    fi
done

echo "-------------------------"
echo "已完成 $N 次循环运行"