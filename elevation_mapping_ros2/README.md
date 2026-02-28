# elevation_mapping_ros2 (RoboSense E1R)

一个可直接运行的 ROS 2 高程映射节点：

- 订阅 `sensor_msgs/msg/PointCloud2` 话题（默认 `/rslidar_points`）。
- 实时构建二维高程网格。
- 发布：
  - `/elevation_map` (`nav_msgs/msg/OccupancyGrid`，按高度归一化到 0~100)
  - `/elevation_map_points` (`sensor_msgs/msg/PointCloud2`，可在 RViz2 以点云显示高程)

## 编译

在工作空间根目录执行：

```bash
colcon build --packages-select elevation_mapping_ros2 --symlink-install
source install/setup.bash
```

## 启动

```bash
ros2 launch elevation_mapping_ros2 e1r_elevation_mapping.launch.py
```

## 回放你的 bag（你给出的 `0228/`）

```bash
ros2 bag play 0228 --clock
```

如果 bag 中 `frame_id` 不是 `map`，可增加静态 tf（示例）：

```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map <your_lidar_frame>
```

## RViz2 显示建议

1. `Fixed Frame` 设为 `map`。
2. 添加 `PointCloud2`，topic 选 `/elevation_map_points`。
3. 添加 `Map`，topic 选 `/elevation_map`（二维俯视）。

## 关键参数（`config/e1r_elevation_mapping.yaml`）

- `resolution`: 网格分辨率（米）
- `map_length_x/map_length_y`: 地图范围（米）
- `min_height/max_height`: 过滤并归一化的高度范围
- `min_points_per_cell`: 单网格最少点数，抑制离散噪声
- `smoothing_factor`: 时间平滑系数，越大越平滑
