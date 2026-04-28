# ROS2 首次运行步骤（Humble + TurtleBot3 + Gazebo Classic）

## 1) 环境准备（在 Ubuntu VM）
```bash
source /opt/ros/humble/setup.bash
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-turtlebot3* \
  python3-colcon-common-extensions
```

## 2) 工作区编译
```bash
cd ~/ta_hdi_ws
colcon build --symlink-install
source install/setup.bash
```

## 3) 启动仿真与节点
终端A：
```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

终端B：
```bash
source ~/ta_hdi_ws/install/setup.bash
ros2 launch ta_hdi_fmp_planner planner_min.launch.py
```

终端C：
```bash
source ~/ta_hdi_ws/install/setup.bash
ros2 launch ta_hdi_fmp_mobile_adapter mobile_adapter.launch.py
```

## 4) 触发一次规划（service）
```bash
ros2 service call /ta_hdi_fmp/plan_path ta_hdi_fmp_msgs/srv/PlanPath2D "{
  start: {x: 0.0, y: 0.0, theta: 0.0},
  goal: {x: 2.0, y: 1.0, theta: 0.0},
  obstacles: [
    {type: 1, cx: 1.0, cy: 0.5, radius: 0.25, width: 0.0, height: 0.0, theta: 0.0}
  ]
}"
```

## 5) 观测话题
```bash
ros2 topic echo /ta_hdi_fmp/planned_path
ros2 topic echo /cmd_vel
```

## 6) 生成实验CSV（占位脚本）
```bash
source ~/ta_hdi_ws/install/setup.bash
ros2 run ta_hdi_fmp_mobile_adapter benchmark_runner
```

生成文件：`benchmark_results_ros2.csv`
