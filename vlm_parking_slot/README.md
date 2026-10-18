# BEV 停车位识别实验（离线图片批处理）

这里并列保留两种离线检测方式：本地 VLM 和传统计算机视觉。检测脚本读取 Gazebo 的 BEV 图片，输出像素坐标和调试图，不修改 ROS 在线车位检测器。只有从 Gazebo 保存图像的 `save_bev_image.py` 需要 ROS。

## 目录

- `images/`：放入从 Gazebo 截取的图片，支持 `.png`、`.jpg`、`.jpeg`、`.bmp`。
- `results/`：脚本生成的叠加图、每张图片的 JSON 和汇总 JSON。
- `run_traditional.py`：传统视觉批处理入口。
- `traditional_parking_detector.py`：颜色分割、白线验证和车位几何检测。
- `run_vlm.py`：本地 VLM 批处理入口。
- `tests/`：传统视觉检测的行为测试。
- `requirements.txt`：Python 依赖。

## 从 Gazebo 保存 BEV 图片

先在终端 1 启动 Gazebo 和 BEV。离线 VLM 采图时关闭在线检测器，减少额外负载：

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch parking_lot_sim parking_lot.launch.py \
 \
  enable_bev_projector:=true \
  enable_parking_slot_detector:=false \
  enable_parking_slot_debug_view:=false \
  enable_parking_slot_map_fusion:=false \
  enable_data_collector:=false
```

再在终端 2 启动图片保存节点：

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
cd ~/ros2_ws/vlm_parking_slot
PYTHONNOUSERSITE=1 python3 save_bev_image.py --output-dir images/run_01
```

在终端 3 保存当前 BEV 帧。每次调用会生成一个新的 PNG，写入 `images/run_01/`：

```bash
source /opt/ros/jazzy/setup.bash
ros2 service call /bev_image_saver/save std_srvs/srv/Trigger "{}"
```

确认服务返回 `success: true` 后，图片已经保存。若提示还没有新帧，等一秒再调用。查看已保存图片：

```bash
ls -lh ~/ros2_ws/vlm_parking_slot/images/run_01/bev_capture_*.png
```

终端 1 的 `Ctrl-C` 会停止 Gazebo；终端 2 的 `Ctrl-C` 只会停止图片保存节点。

## 传统视觉识别

当前 Gazebo 场景里停车位地面呈蓝灰色、车道呈绿色、分隔线呈白色。传统检测先按颜色分出车位地面，再用白线支持度、矩形度和已知车位尺寸过滤候选，输出适合斜置车位的四角多边形。它不需要加载模型或使用 GPU，阈值针对当前仿真材质。

当前 BEV 分辨率是 `0.02 m/像素`，车位宽度设置为 `2.8 m`，因此检测器用约 `140 像素`作为短边尺寸参考，并允许一定偏差。若修改 BEV 范围、分辨率或仿真车位尺寸，需要相应调整 `DetectorConfig`。

处理刚保存的这批图片：

```bash
cd ~/ros2_ws/vlm_parking_slot
source .venv/bin/activate
python run_traditional.py \
  --input-dir images/run_01 \
  --output-dir results/traditional_run_01
```

输出包括：

- `*_traditional.jpg`：绿色为接受的车位，橙色为被拒绝的候选；红点为四角。触边或标线不足的部分车位会在 JSON 中标记为不完整。
- `*_slot_mask.png`：蓝灰色车位地面分割结果。
- `*_white_lines.png`：白色标线分割结果。
- `*.json` 和 `all_results.json`：四角像素坐标、尺寸几何、边线支持度和遮挡/裁切标记。

查看结果：

```bash
xdg-open ~/ros2_ws/vlm_parking_slot/results/traditional_run_01
```

测试传统视觉检测器：

```bash
cd ~/ros2_ws/vlm_parking_slot
source .venv/bin/activate
python -m unittest discover -s tests -v
```

## VLM 识别

VLM 保留为对照实验。它输出轴对齐矩形框，适合检查语义识别效果，但当前图片上框的位置不够可靠；不要把模型返回的框当成准确车位边界。

### 安装依赖

建议使用单独虚拟环境，但复用系统中已有的 PyTorch：

```bash
cd ~/ros2_ws/vlm_parking_slot
python3 -m venv .venv --system-site-packages
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

如果当前环境已经能导入 PyTorch，不需要重复安装 PyTorch。Qwen 官方示例使用 `transformers`、`accelerate` 和 `qwen-vl-utils`。

### 准备本地模型

默认使用 `Qwen2.5-VL-3B-Instruct`。在 RTX 4080 Laptop 12 GB 显存上先从 3B 开始，避免直接使用更大的模型。

例如把模型下载到 `~/models/Qwen2.5-VL-3B-Instruct`：

```bash
mkdir -p ~/models/Qwen2.5-VL-3B-Instruct
hf download Qwen/Qwen2.5-VL-3B-Instruct \
  --local-dir ~/models/Qwen2.5-VL-3B-Instruct
```

如果机器不能访问 Hugging Face，可以在其他机器下载后把整个模型目录复制过来。

### 放入截图

把十几张 Gazebo BEV 截图复制到本目录的 `images/`，例如：

```bash
cp ~/截图目录/*.png ~/ros2_ws/vlm_parking_slot/images/
```

图片最好保持相同的 BEV 视角和分辨率。VLM 基线输出轴对齐矩形框；传统视觉入口输出四角多边形。

### 批量识别

```bash
cd ~/ros2_ws/vlm_parking_slot
source .venv/bin/activate
python run_vlm.py \
  --input-dir images \
  --output-dir results \
  --model-path ~/models/Qwen2.5-VL-3B-Instruct
```

也可以直接使用 Hugging Face 模型名，让 Transformers 首次运行时下载：

```bash
python run_vlm.py \
  --input-dir images \
  --output-dir results \
  --model-path Qwen/Qwen2.5-VL-3B-Instruct
```

### 输出

对每张输入图片，脚本会生成：

- `results/<原文件名>_boxes.jpg`：画出停车位框和编号的图片。
- `results/<原文件名>.json`：该图片的识别结果，坐标是原图像素坐标，格式为 `[x1, y1, x2, y2]`。
- `results/all_results.json`：所有图片的汇总结果。
- `results/<原文件名>.raw.txt`：模型没有按 JSON 返回时保存原始回答，方便检查。

如果某张图片没有足够把握的停车位，结果中的 `slots` 会为空，不会强行画框。

### 参数

```text
--input-dir          输入图片目录，默认 images
--output-dir         输出目录，默认 results
--model-path         本地模型目录或 Hugging Face 模型名
--max-new-tokens     每张图最多生成的 token 数，默认 512
--coordinate-scale   坐标解释方式：auto、pixels、1000，默认 auto
--limit              只处理前 N 张图片，默认处理全部
```

`auto` 会优先按原图像素解释；如果模型返回的坐标超过图像大小，则按 0 到 1000 的归一化坐标转换。

## 当前边界

两种识别方式都只处理离线 BEV 图像，不发布 ROS 话题、不做 TF 转换、不控制 Gazebo 小车，也不替换现有在线检测器。传统视觉当前利用仿真里的固定材质颜色；若改变地面材质或 BEV 色彩，需要重新检查分割阈值。VLM 和传统视觉的输出都应先通过叠加图人工核对，再考虑转换到车辆坐标或接入在线节点。
