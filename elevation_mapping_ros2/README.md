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

1. 默认参数下，`Fixed Frame` 设为 `map`（因为默认 `map_frame_from_cloud: false`）。
2. 添加 `PointCloud2`，topic 选 `/elevation_map_points`。
3. 添加 `Map`，topic 选 `/elevation_map`（二维俯视）。
4. 如果你改成 `map_frame_from_cloud: true`，则 `Fixed Frame` 必须改成雷达 frame（如 `rslidar`），或提供 `map -> rslidar` 的 TF。

## 关键参数（`config/e1r_elevation_mapping.yaml`）

- `resolution`: 网格分辨率（米）
- `map_length_x/map_length_y`: 地图范围（米）
- `min_height/max_height`: 点云高度过滤范围
- `aggregation_method`: `max`(推荐粮堆表面) / `mean`
- `min_points_per_cell`: 单网格最少点数，抑制离散噪声
- `smoothing_factor`: 时间平滑系数，越大越平滑
- `auto_contrast`: 自动按当前地图高度动态拉伸到 0~100，让 RViz 颜色更明显
- `visualization_min_height/visualization_max_height`: 关闭 `auto_contrast` 时使用的固定显示范围
- `map_frame_from_cloud`: `false` 时输出固定为 `map_frame`（默认，避免 RViz Message Filter 因 TF 缺失而丢帧）；`true` 时输出使用输入点云 frame


## 常见问题修复

- 你反馈的“/elevation_map 错误”：通常由 frame 不一致造成，此版本默认改为 `map_frame_from_cloud: false`，优先避免你遇到的 RViz 丢帧问题。
- 你反馈的“颜色不明显”：默认已改为 `auto_contrast: true`，会按实时高度范围自动增强对比度。

- 若看到 `Message Filter dropping message ... queue is full`，99% 是 Fixed Frame 与消息 frame 之间没有可用 TF。请优先确认 `map_frame_from_cloud` 配置与 RViz Fixed Frame 一致。
