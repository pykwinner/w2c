# w2c

ROS 工程集合，包含两个子项目：

## 子项目

### 1. clean_robot_code
服务机器人竞技赛清洁相关 Demo（地面清洁、桌面清洁）。

**包含包：**
- `clean_robot` — 服务机器人竞技 A
- `clean_desktop_robot` — 服务机器人竞技 B
- `carry_robot` — 服务机器人竞技 C
- `lift_robot`、`oil_check_robot`、`shoot_robot` 等

### 2. upros_class_code
Upros 机器人 ROS 代码库，包含导航、机械臂、控制、视觉等模块。

**主要包：**
- `upros_description` / `s2a_description` — 机器人 URDF/Srdf 描述
- `upros_moveit` / `s2a_moveit_config` — MoveIt 运动规划
- `upros_navigation` / `w2u_navigation` — 导航
- `upros_driver` / `upros_hardware` — 驱动与硬件接口
- `upros_cv` / `upros_depth_vision` — 视觉
- `upros_arm` / `zoo_arm` — 机械臂
- `teb_local_planner` — TEB 局部规划器
- `costmap_prohibition_layer` — 代价地图禁行层
- `upros_imu_filter` / `upros_lidar_filter` — 传感器滤波
- `gui` / `upros_gui` — 图形界面

## 目录结构

```
w2c/
├── README.md
├── src_clean_robot_code/    # 清洁机器人竞赛代码
│   ├── clean_robot/
│   ├── clean_desktop_robot/
│   ├── carry_robot/
│   └── ...
└── src_upros_class_code/    # Upros 机器人主代码库
    ├── upros_description/
    ├── upros_moveit/
    ├── upros_navigation/
    └── ...
```

## 安装

```bash
# 安装依赖
rosdepc install --from-paths src_upros_class_code --ignore-src --rosdistro=noetic
rosdepc install --from-paths src_clean_robot_code --ignore-src --rosdistro=noetic

# 编译
catkin_make
```

## 作者

宋刚
