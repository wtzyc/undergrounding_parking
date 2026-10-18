# 地下停车场仿真与车位识别

这是 ROS 2 Jazzy + Gazebo Sim 的地下停车场实验工作区。在线 BEV 车位检测使用 C++/OpenCV 传统视觉；另有独立的离线 VLM 和 Python 传统视觉工具，用于处理已保存的 Gazebo 截图。

## 目录

- `src/world/track_world.sdf`：停车场仿真场景。
- `src/models/orange_audibot/`：车辆和传感器模型。
- `src/parking_lot_sim/`：Gazebo/ROS 2 启动、BEV、键盘控制、数据采集和 SLAM 工具。
- `src/parking_lot_perception/`：C++ BEV 车位检测、LaserScan 柱子候选检测与语义地图融合。
- `src/parking_lot_core/`：C++ 地图数据结构、几何和时间同步基础模块。
- `src/parking_lot_navigation/`：Nav2 阿克曼车辆参数、行为树和手动导航启动配置。
- `vlm_parking_slot/`：与 ROS 分离的本地 VLM 截图批处理模块，具体步骤见该目录的 README。

## 编译

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
PYTHONNOUSERSITE=1 colcon build --symlink-install
source install/setup.bash
```

运行已有测试：

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon test --packages-select parking_lot_core parking_lot_perception parking_lot_navigation
colcon test-result --verbose
PYTHONPATH=src/parking_lot_sim:$PYTHONPATH python3 -m pytest -q src/parking_lot_sim/test
```

## 启动 Gazebo、BEV 和在线车位检测

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch parking_lot_sim parking_lot.launch.py
```

默认启动 Gazebo 停车场、ROS/Gazebo 话题桥、BEV 投影、C++ 车位检测、车位和柱子语义融合、LaserScan 柱子候选检测及车位调试图窗口。柱子检测需要同时启动下面的 `slam_mapping.launch.py`，由它提供雷达到车体和 `map` 的 TF。车位仍按 10 帧稳定条件确认；柱子需有 10 次稳定观测，中心位置散布在 0.15 米内，最多容忍 5 帧暂时漏检。确认后的柱子会以 map 坐标地标留在 RViz 中，离开雷达视野后不会消失。上帝相机保持 800×800 分辨率并以 10 Hz 更新；RGBD、左、右、前、后视相机默认休眠，只有启用数据采集或 rosbag 录制时才启动前视相机及对应桥接。数据采集和 rosbag 录制默认关闭。

需要驾驶时，另开终端：

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run parking_lot_sim keyboard_teleop
```

键盘控制：`w/s` 调整前进/后退速度，`a/d` 转向，空格停车并回正，`x` 反转当前速度，`q/e` 调速度步长，`z/c` 调转向步长，`Ctrl-C` 退出。

## 在线检测结果

常用话题：

- `/camera_god/bev_image`：BEV 图像，在线 C++ 检测器的输入。
- `/camera_god/parking_slot_debug_image`：画有实时检测结果的 BEV 调试图，启动时默认打开 rqt 窗口。
- `/semantic/parking_slot_candidates`：相对于车辆 `base_footprint` 的车位观测。
- `/semantic/parking_slots_map`：`parking_lot_core` 版本化语义地图，确认后的车位、柱子和虚拟道路节点均在 `map` 坐标；虚拟节点包含邻居 ID、相对位姿和代价，`edges` 仍为空。
- `/semantic/parking_slot_markers`：已确认车位在 `map` 坐标系下的 RViz 标记，包含完整车位轮廓、中心点和节点编号。
- `/semantic/pillar_candidates`：由 `/scan` 几何聚类得到的柱子候选，位置相对于 `base_footprint`；状态、扫描时间戳和来源雷达 frame 一并发布。
- `/semantic/pillar_candidate_markers`：橙色临时柱子候选标记，可用于检查车辆相对位置。
- `/semantic/pillar_landmark_markers`：已稳定确认、固定在 `map` 坐标系下的柱子地标；车辆离开后仍会保留显示。
- `/semantic/virtual_road_node_markers`：根据确认车位和柱子生成的虚拟道路节点；黄色为车位入口节点，紫色为柱子的可通行方向节点。
- `/camera_god/bev_status`：BEV 投影状态及坐标映射信息。

融合节点默认从 `~/ros2_ws/runtime/maps/parking_lot_semantic_map.json` 加载已有语义地图（文件不存在时从空地图开始）。已有的确认节点会在重启后继续显示；提高连续帧门槛不会清除旧节点。车位长轴相差 180° 时视为相同朝向；中心相距不超过 1.2 米、朝向差不超过 10°且尺寸接近的重叠车位会合并，约 3 米间隔的相邻真实车位仍保持独立。旧 JSON 中符合这些条件的重复车位会在启动加载时合并，之后需再次调用保存服务，才会把清理结果写回文件。需要从空地图验证时，请给 `semantic_map_path` 指定一个尚不存在的新 JSON 路径。保存当前地图：

```bash
ros2 service call /semantic/parking_slots_map/save std_srvs/srv/Trigger "{}"
```

这个服务保存确认后的车位、柱子、虚拟道路节点及其邻居关系 JSON；当前仍不创建 `edges` 数组。SLAM 的 `/map` 栅格地图需要单独通过 `/slam_toolbox/save_map` 保存。两者使用 `map` 坐标系，但不是同一个文件或数据类型。在线 SLAM RViz 配置会把车位、柱子和虚拟道路节点叠加显示在 `/map` 上。

也可以在启动时指定地图文件路径：

```bash
ros2 launch parking_lot_sim parking_lot.launch.py \
  semantic_map_path:=/home/$USER/ros2_ws/runtime/maps/parking_lot_semantic_map.json
```

也可以手动查看调试图：

```bash
ros2 run rqt_image_view rqt_image_view /camera_god/parking_slot_debug_image
```

在线车位检测根据 BEV 的蓝灰色车位区域、白色标线支持度和矩形尺寸生成候选，并在图像边缘裁切时用车位真实尺寸补全中心和轮廓。柱子检测把相邻有效 LaserScan 点聚成簇，按观测宽度筛选，再依据 1.2 米柱宽估算中心；同一帧内相距不超过 0.6 米的重复候选只保留置信度较高者，多帧融合后相距不超过 0.8 米的重复轨迹会合并。TF 将位置从雷达 frame 转到 `base_footprint`，融合节点再按扫描时间附近的 TF 转到 `map`。新柱子经过 10 次稳定观测后才成为地图地标；确认后即使后续扫描漏掉它，也会继续显示并随语义地图保存/加载。确认地标会进一步生成虚拟道路节点：车位节点沿车位纵向朝配置的车道中心延伸，默认延伸距离从 1.8 米增加 15% 为 2.07 米；柱子不再根据车位方向推断道路方向，而是检查东、北、西、南四个固定方向。只有确实有候选射线进入相邻车位时，才把该车位视为这根柱子的相邻车位；随后删除朝向车位的节点，并从剩余候选中再删除一个距离该车位最近的节点。剩余两个柱子节点中，只有方向与车位延伸方向一致的那个会投影到与车位虚拟节点相同的平齐线上，另一个保持原方向；两者到柱子的距离统一，且仍会做碰撞和重复过滤。生成虚拟节点后，系统按最大距离 5.0 米为虚拟节点建立双向邻居关系，计算每个邻居在源节点坐标系中的相对 `x/y/yaw`、方向和欧氏代价；这里只填充 `neighbors`，不创建 `edges`。二维雷达的短墙段仍可能产生假候选，因此多帧稳定确认是进入地图前的过滤步骤。

虚拟节点的默认参数是：车位边界向车道延伸 2.07 米（原 1.8 米增加 15%），柱子每个候选方向延伸 2.0 米，车道中心为 `(0, 0)`，用于判断相邻车位的最大距离为 8.0 米。停车场布局不同的时候，可以在启动时调整：

```bash
ros2 launch parking_lot_sim parking_lot.launch.py \
  slot_lane_extension_m:=2.07 \
  pillar_extension_m:=2.0 \
  lane_center_x:=0.0 \
  lane_center_y:=0.0 \
  pillar_slot_adjacency_max_distance_m:=8.0 \
  road_node_max_neighbor_distance_m:=5.0
```

当前已经完成虚拟节点生成和邻居相对位姿；A* 仍未实现。

检查柱子候选时，先启动 Gazebo 主流程，再在另一个终端启动 SLAM（后者提供雷达到车体的 TF）：

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch parking_lot_sim slam_mapping.launch.py enable_rviz:=true
```

查看最近一帧候选 JSON：

```bash
ros2 topic echo /semantic/pillar_candidates --once --field data --full-length
```

`status` 为 `ok` 表示当前扫描有候选，`no_candidates` 表示扫描有效但没找到符合尺寸的簇，`tf_unavailable` 表示雷达到 `base_footprint` 的 TF 不可用。RViz 中橙色 `Pillar Candidates` 是当前帧结果；蓝色 `Confirmed Pillar Landmarks` 是经过稳定确认的 map 地标，会保留到语义地图中。用现有服务保存时，回复会分别报告车位和柱子数量。暂时关闭候选检测可在 Gazebo 启动命令后加 `enable_pillar_detector:=false`。

目前车位融合使用观测当时的 `map → base_footprint` 位姿。已经确认的车位不会因为 SLAM 后续优化历史扫描而自动重算；当前时刻 `map → odom` 的变化也不能直接套到所有旧车位上，因为不同位置的历史扫描可能被分别修正。若 RViz 中旧车位与栅格地图不再对齐，应先核对同一时刻的 Gazebo 真值、里程计和 SLAM 位姿，并在闭环测试中检查重访车位的位置。一次位姿一致性实测见 [诊断报告](../runtime/diagnostics/pose_consistency_summary_20261013.md)。

单独关闭某个功能时可传启动参数，例如：

```bash
ros2 launch parking_lot_sim parking_lot.launch.py \
  enable_parking_slot_debug_view:=false
```

## 自动探索功能已移除

本项目不再启动或运行自动前沿探索器，车辆建图时使用键盘手动控制。保留的 `parking_lot_navigation` 仅提供 Nav2 参数、行为树和普通导航启动配置，后续如需要手动发送导航目标仍可使用它；它不会自动驾驶车辆，也不会自动选择前沿。

常用的手动建图流程是：先启动 `parking_lot.launch.py`，再在另一个终端启动 `slam_mapping.launch.py`，最后运行 `keyboard_teleop` 驾驶车辆。这样可以避免多个 Gazebo 或 `/clock` 实例同时运行。

## 在线 SLAM 建图

先安装 SLAM Toolbox：

```bash
sudo apt install ros-jazzy-slam-toolbox
```

Gazebo 仿真启动后，在另一个终端运行：

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch parking_lot_sim slam_mapping.launch.py enable_rviz:=true
```

RViz2 会在线显示 `/map`、`/scan` 和 TF。查看地图或车辆 TF：

```bash
ros2 topic echo /map --once
ros2 run tf2_ros tf2_echo map base_footprint
```

保存栅格地图：

```bash
mkdir -p ~/ros2_ws/runtime/maps
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "{name: {data: '$HOME/ros2_ws/runtime/maps/parking_lot_map'}}"
```

## 可选：采集图片或录制 rosbag

启动图片采集节点（保存前视图像及对应 metadata）：

```bash
ros2 launch parking_lot_sim parking_lot.launch.py \
```

默认保存到 `~/ros2_ws/dataset/raw/`。调整采集间隔示例：

```bash
ros2 launch parking_lot_sim parking_lot.launch.py \
  enable_data_collector:=true collection_save_interval:=1.0
```

如需录制完整传感器话题供回放，使用一个新的输出目录：

```bash
ros2 launch parking_lot_sim parking_lot.launch.py \
  enable_rosbag_recording:=true \
  bag_output_dir:=~/ros2_ws/dataset/bags/run_01
```

检查与回放：

```bash
ros2 bag info ~/ros2_ws/dataset/bags/run_01
ros2 launch parking_lot_sim replay_parking_lot.launch.py \
  bag_path:=~/ros2_ws/dataset/bags/run_01
```

## 离线 VLM 车位识别

BEV 离线识别实验包括本地 VLM 和传统视觉两种入口。传统视觉通过仿真车位地面的颜色、白色标线和矩形几何输出车位四角；VLM 输出矩形框。按 [模块说明](../vlm_parking_slot/README.md) 从 Gazebo 保存 BEV 图片并运行对照。带标注图和像素坐标写入 `vlm_parking_slot/results/`。这两种离线入口都不发布 ROS 车位话题，也不替换在线 C++ 检测器。

这个模块用于先检查本地 VLM 在截图上的识别效果；它目前不向 ROS 发布结果，也不把检测框自动转换到语义地图坐标。
