Terminal A: Start Vehicle Client
```
python3 src/launch/simple_racing/scripts/launch_ossdc.py --env svl.env
```
Terminal B: Start Self driving stack
```
colcon build --packages-up-to simple_racing
source install/setup.bash
ros2 launch simple_racing simple_racing.launch.py params_file:=src/launch/simple_racing/params/simple_racing.yml
```