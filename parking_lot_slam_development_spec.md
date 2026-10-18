# 地下停车场语义 SLAM 与拓扑导航开发 Spec

## 题目目标

在 Gazebo 地下停车场中，让车辆通过相机、激光雷达和里程计：

1. 识别停车位、柱子和区域字母；
2. 估计它们相对于车辆的二维位置和方向；
3. 使用里程计和语义地标建立度量 SLAM 地图；
4. 根据停车位、柱子和车道几何生成虚拟道路节点；
5. 保存虚拟节点及邻接关系；
6. 后续直接在这些离散节点上使用 A*，实现从一个停车位到另一个停车位的路径搜索。

第一阶段只验证一个区域，例如 B 区；使用闭环驾驶路线；暂不自动控制车辆，也不处理完整碰撞规划。

## 已知信息

### README、代码和运行方式

项目主要内容：

- `src/world/track_world.sdf`：停车场世界文件；
- `src/models/orange_audibot/model.sdf`：车辆模型；
- `src/parking_lot_sim/launch/parking_lot.launch.py`：启动文件；
- `src/parking_lot_sim/parking_lot_sim/nodes/gray_pillar_detector.py`：灰度柱子检测器；
- `src/parking_lot_sim/parking_lot_sim/nodes/data_collector.py`：数据采集器；
- `runtime/parking_lot_area_map.json`：运行时地图文件。

基础运行：

```bash
cd /home/wt/ros2_ws
PYTHONNOUSERSITE=1 colcon build --symlink-install
source install/setup.bash
ros2 launch parking_lot_sim parking_lot.launch.py
```

可选功能：

```bash
ros2 launch parking_lot_sim parking_lot.launch.py enable_data_collector:=true
ros2 launch parking_lot_sim parking_lot.launch.py enable_gray_pillar_detector:=true
ros2 launch parking_lot_sim parking_lot.launch.py enable_gray_pillar_detector:=true
```

已有话题包括：

- `/camera_front/image`、`/camera_front/camera_info`；
- `/camera_god/image`、`/camera_god/camera_info`；
- `/camera_left/image`、`/camera_right/image`、`/camera_rear/image`；
- `/scan`、`/imu`、`/navsat`；
- `/model/my_orange_car/odometry`；
- `/world/luxury_parking_lot_expanded/pose/info`；
- `/cmd_vel`。

已有节点包括：

- `data_collector`；
- `gray_pillar_detector`；
- `keyboard_teleop`。

### 世界和车辆模型

世界文件包含 A～H 共 8 个区域。每个区域当前有 8 个停车位，排列为 2 排 × 4 列，并有 8 根柱子。场景中只有一辆 `my_orange_car`。

车辆包含前、左、右、后相机、一个额外 RGB-D 相机、二维激光雷达、IMU、GPS 和里程计。前视相机是普通 RGB 相机，分辨率 1280×720，水平视场角约 1.65 rad，更新频率 10 Hz。雷达为 360 点二维扫描，量程约 0.4～15 m，更新频率 10 Hz。

### 已有检测输出

当前保留的灰度柱子检测器输出 `pillar_candidate`，不能保证结果一定是柱子。后续专用检测器需要统一输出目标类型、置信度、检测框、方位角、距离和 `base_footprint` 下的相对位置。

当前候选检测链路没有严格时间同步、目标朝向、跨帧跟踪、唯一地标 ID 或 SLAM 关联。

### 数据和测试现状

当前已有约 571 张图像和 metadata，但 metadata 只有雷达摘要，没有完整雷达扫描；已有数据中的 `truth_pose` 基本为空，不能直接用于完整激光 SLAM 回放。

未找到项目测试用例。`setup.py` 仅声明了 pytest 测试依赖。当前工作区没有 SLAM 节点；环境中也没有发现已安装的 `slam_toolbox`、Cartographer 或 Nav2 包。现有 `runtime/parking_lot_area_map.json` 没有实际区域、节点和边。

## 需求拆解

| 功能 | 要做什么 | 影响模块 | 完成判断 |
|---|---|---|---|
| 原始数据采集 | 录制 `/scan`、里程计、图像、相机参数和时间戳 | launch、采集器、rosbag | 能完整回放同一段驾驶过程 |
| 区域识别 | 识别 A～H 区域字母 | 检测模型、相机 | 正确识别区域并参与地标 ID |
| 停车位检测 | 在前视局部 BEV 中识别入口线或四角点 | BEV 和车位检测模块 | 输出中心、朝向、入口端点和尺寸 |
| 柱子检测 | 在原始透视图中检测柱子 | 检测器、雷达融合 | 输出柱子中心相对位姿 |
| 雷达融合 | 将图像方位和雷达距离转换到车辆坐标系 | 感知融合模块 | 坐标方向、安装偏移和时间戳正确 |
| 地标 ID | 形成类似 `B区-停车位-2-1` 的稳定 ID | 语义关联模块 | 重复观测不会持续产生重复 ID |
| 度量 SLAM | 使用二维激光和里程计建立度量轨迹 | `slam_toolbox` 适配层 | 闭环后轨迹和地标位置保持一致 |
| 语义回环 | 用重复看到的柱子、车位辅助回环和位姿优化 | 语义地图层 | 重复观测能匹配到已有地标 |
| 虚拟道路节点 | 从停车位和柱子向车道方向延伸生成节点 | 虚拟节点生成模块 | 节点不落入已识别车位或柱子区域 |
| 道路邻接 | 保存可通行邻居、方向、相对位姿和代价 | 地图数据层 | 邻接关系能表达分叉和方向 |
| 地图保存 | 保存语义地标、虚拟节点和边 | JSON/YAML 地图文件 | 重启后可重新加载 |
| 拓扑定位 | 将车辆匹配到虚拟节点或道路边 | 定位模块 | 输出节点/边 ID、边内位置和朝向 |
| A* 搜索 | 直接在虚拟节点集合上搜索 | 路径规划模块 | 能找到两个停车位之间的节点路径 |
| 数据评价 | 计算位置误差、朝向误差、回环和搜索成功率 | 离线评估工具 | 生成可重复的数值报告 |

## 实现思路

### 总体分层

```text
传感器数据
    ↓
目标检测与几何估计
    ↓
语义地标关联
    ↓
度量 SLAM 与虚拟节点生成
    ↓
拓扑定位与 A* 搜索
```

度量地图和虚拟节点地图同时保存，但不再额外建立一套栅格搜索地图。A* 直接使用虚拟节点及其邻接边。

### 停车位感知

第一阶段使用模型已有的 `camera_god` 作为俯视相机模拟局部 BEV：读取 `/camera_god/image` 和 `/camera_god/camera_info`，使用固定的 15 米安装高度和地面平面将图像转换到 `base_footprint` 局部 BEV，在 BEV 中检测入口线或四角点，并恢复车位中心、纵向方向、入口端点、长宽和置信度。实验重点是语义地标和拓扑导航，因此暂不投入前视相机标定和多相机拼接。

停车位不直接使用普通矩形框中心作为最终位姿。入口线或角点几何更适合恢复车位方向和位置。

### 柱子感知

在原始透视图中使用 GroundingDINO 或训练后的专用模型检测柱子；由检测框中心计算水平角，在对应雷达角度窗口内获取距离，补偿雷达安装位置并转换到 `base_footprint`。柱子在地图中表示为二维中心地标。

### GroundingDINO

GroundingDINO 只用于离线候选生成：

```text
GroundingDINO 候选框
    ↓
人工修正少量样本
    ↓
训练专用车位/柱子模型
    ↓
在线运行专用模型
```

### 地标数据结构

```text
LandmarkNode
- id
- type: parking_slot | pillar
- zone
- row
- column
- pose: x, y, yaw
- geometry: length, width, entrance_pose
- confidence
- observation_count
- status: candidate | confirmed
- source
```

柱子只要求中心位置，`yaw` 可以为空。停车位必须保存入口方向。

### 语义编号

编号采用：

```text
区域 + 类型 + 排号 + 列号
```

例如：

```text
B-PS-2-1
B-PILLAR-2-2
```

第 1 排由区域入口侧确定，列号沿区域局部左右方向排列。入口方向通过车位排列和车道侧空隙推断，不使用 Gazebo 世界真值作为在线输入。

### 度量 SLAM

建议使用适配层连接 `/scan`、里程计和二维 SLAM：

```text
/scan + /odom
        ↓
二维激光 SLAM
        ↓
map → odom → base_footprint
```

初始基线使用 `slam_toolbox`。语义地标先在上层维护：里程计提供短期运动约束，激光 SLAM 提供基础轨迹和栅格地图，柱子和车位提供语义回环和地标一致性约束。

### 虚拟道路节点

停车位从入口或中心沿纵向轴向车位外侧单向延伸。柱子根据附近停车位确定禁行方向，再向另外三个方向生成候选分支。

生成阶段只做基础排除：不落入已识别停车位区域、不落入柱子区域，遇到边界、已存在道路图或最大距离时停止。当前阶段不做完整动态碰撞规划。

```text
RoadNode
- id
- anchor_id
- pose_map: x, y, yaw
- node_type: road | parking_anchor | pillar_anchor
- neighbors:
    - neighbor_id
      relative_x
      relative_y
      relative_yaw
      direction
      cost
- confidence
- source
```

每个节点同时保存全局 `map` 位姿和到邻居节点的相对位姿。边为有向边，保存方向、相对位姿和代价。

### 多帧关联

```text
单帧检测
    ↓
临时候选
    ↓
多帧位置和朝向一致性检查
    ↓
确认地标
    ↓
生成或更新虚拟节点
```

重复观测依据语义 ID、所属锚点、拓扑位置、相对距离和 SLAM 优化后的全局位置进行匹配。

### A* 搜索

A* 不重新建立栅格，也不临时连接任意节点。起点和终点是停车位节点，搜索空间是全部虚拟节点和已确认邻接边。边代价由长度、转弯、反向和方向切换组成，输出虚拟节点 ID 序列。连续轨迹、车辆运动学和碰撞检查由后续导航模块负责。

### 地图保存

地图文件至少保存：

```text
map_metadata
landmarks
road_nodes
edges
coordinate_frame
parameters
evaluation_metadata
version
```

建议继续使用版本化 JSON，兼容现有 `runtime/parking_lot_area_map.json` 的思路。

## 测试与验收

### 构建和测试命令

```bash
cd /home/wt/ros2_ws
PYTHONNOUSERSITE=1 colcon build --symlink-install
source install/setup.bash
colcon test --packages-select parking_lot_sim \
  --event-handlers console_direct+
colcon test-result --verbose
python3 -m pytest
```

当前没有实际测试目录；实现后应优先补充 pytest 单元测试，并保持上述命令可用。

### 运行验证

```bash
ros2 launch parking_lot_sim parking_lot.launch.py
ros2 launch parking_lot_sim parking_lot.launch.py enable_data_collector:=true
ros2 run parking_lot_sim keyboard_teleop
```

### 关键测试场景

1. 单个停车位：恢复入口线/角点、中心、方向和入口端点；
2. 单根柱子：图像检测与雷达距离融合正确；
3. 重复观测：同一地标不产生重复正式节点；
4. 虚拟节点：不落入已识别停车位或柱子区域；
5. 闭环 SLAM：返回原区域后地标和轨迹保持一致；
6. A*：从一个停车位找到另一个停车位的合法节点序列；
7. 地图持久化：保存后重新加载节点、边和语义 ID。

### 初始验收标准

- 柱子中心误差不超过约 0.3 m；
- 车位中心误差不超过约 0.3 m；
- 车位朝向误差不超过约 10°；
- 错误合并或重复建点率不超过约 10%；
- 闭环后主要地标仍能匹配；
- 起点和终点明确时，A* 能找到有效节点路径；
- 地图可保存和重新加载；
- B 区闭环测试可重复运行。

## 不做什么

第一阶段不做：

- 识别其他停车车辆；
- 使用 Gazebo 真值参与在线建图；
- 覆盖 A～H 全部区域；
- 自动驾驶和自动泊车；
- 完整动态障碍物避碰；
- 三维 SLAM；
- BEVFormer 等重型学习式 BEV；
- 将 GroundingDINO 作为最终在线模型；
- 重新划分用于 A* 的栅格地图；
- 在 A* 阶段临时连接任意节点；
- 修改世界布局和车辆模型。

## 风险点

- 相机、雷达和车辆坐标系方向可能不一致；
- 现有检测器没有严格图像/雷达时间同步；
- 雷达安装位置不能直接当作车体原点；
- 上帝摄像头高度和姿态误差会影响车位角点；
- GroundingDINO 伪标注可能误检车位线、阴影或其他灰色物体；
- 柱子遮挡可能导致三方向延伸生成错误道路节点；
- 同一区域多个柱子和车位容易发生错误合并；
- 停车位入口方向判断错误会导致道路方向错误；
- 虚拟节点过密会增加 A* 搜索量，过稀会降低路径精度；
- `slam_toolbox` 当前未安装，ROS 2 Jazzy 依赖需要单独验证；
- 当前没有实际测试用例，需避免破坏现有入口、话题和启动参数；
- 世界文件中的固定绝对路径资源可能影响环境迁移；
- 现有数据无法直接完成完整雷达 SLAM 回放，需要重新录制 rosbag。
