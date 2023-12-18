# Compile, buod and run the program
```
rosdep install -i --from-path py_pubsub/py_pubsub --rosdistro humble -y
colcon build --packages-select py_pubsub
source install/setup.bash
ros2 run py_pubsub camera
```

# Config camera
```
v4l2-ctl -d /dev/video0 --all
```

# Publish an array of 3 Int8 on the topic camera/hsv_low once
```
ros2 topic pub --once camera/hsv_low std_msgs/msg/Int8MultiArray "{data : [99,91,30]}"
```
