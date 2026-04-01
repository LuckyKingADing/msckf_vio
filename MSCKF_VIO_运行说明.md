# msckf_vio 运行说明

> 环境：Ubuntu 22.04 + ROS Noetic  
> 工作空间：`~/leetcode/catkin_ws`  
> 数据集：EuRoC Machine Hall `MH_01_easy`

---

## 系统架构：两个核心节点

msckf_vio 系统由 **两个 ROS Nodelet 节点** 协同工作，缺一不可：

### `image_processor` 节点（前端：视觉特征处理）

| 项目 | 说明 |
|------|------|
| **作用** | 接收双目图像 + IMU 数据，进行**特征检测、光流追踪、双目匹配、外点剔除**，输出追踪好的特征点 |
| **订阅话题** | `/imu0`（IMU）、`/cam0/image_raw`、`/cam1/image_raw`（双目图像） |
| **发布话题** | `features`（CameraMeasurement：特征归一化坐标）、`tracking_info`（追踪统计）、`debug_stereo_image`（调试图像） |
| **核心算法** | FAST 角点检测 → KLT 光流追踪 → 2-point RANSAC 外点剔除 → 网格化特征管理 |

### `vio` 节点（后端：MSCKF 滤波器）

| 项目 | 说明 |
|------|------|
| **作用** | 接收 IMU 数据 + 前端输出的特征观测，运行 **EKF 滤波器**，输出 6DOF 位姿估计 |
| **订阅话题** | `/imu0`（IMU）、`image_processor/features`（来自前端的特征） |
| **发布话题** | `odom`（nav_msgs/Odometry：**主输出位姿**）、`feature_point_cloud`（3D 路标点云） |
| **核心算法** | IMU 预积分 → 状态增广 → 特征三角化 → EKF 量测更新 → 滑动窗口管理 |

### 两者的关系

```
双目图像 ──→ [image_processor] ──features──→ [vio] ──→ 位姿输出 (odom)
                   ↑                            ↑
                 /imu0                        /imu0
```

- **前端**负责"看"（从图像中提取和追踪特征点）
- **后端**负责"算"（将特征观测与 IMU 融合，估计位姿）
- 两者通过 `features` 话题连接，**必须同时运行**

### 启动方式

**不需要分别启动**。执行 `roslaunch msckf_vio msckf_vio_euroc.launch` 会**自动同时启动两个节点**：
- `msckf_vio_euroc.launch` 内部 include 了 `image_processor_euroc.launch`（启动前端）
- 同时启动 `vio` nodelet（后端）

如果只启动 `image_processor_euroc.launch`，则只有前端在跑（发布特征），没有后端做位姿估计，**不会输出 odom**。

---

## 每次运行前的准备（只需一次，永久生效）

建议将以下两行加入 `~/.bashrc`，避免每次手动 source：

```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
echo "source ~/leetcode/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 运行步骤（需要 4 个终端）

### 终端 1 — 启动 ROS Master

```bash
source /opt/ros/noetic/setup.bash
source ~/leetcode/catkin_ws/devel/setup.bash
roscore
```

> 看到 `started core service [/rosout]` 后保持此终端不动，继续开其他终端。  
> **注意**：如果提示 `roscore cannot run as another roscore/master is already running`，说明 roscore 已在后台运行，**直接跳过此步骤**，继续启动终端2即可。
> 如何kill
> 方法1：直接 kill（推荐） 依次执行
> pkill -f roscore
> pkill -f rosmaster
> 方法2：查找 PID 再 kill
> ps aux | grep roscore
> 找到 PID 后：sudo kill -9 <PID>

---

### 终端 2 — 启动 msckf_vio 算法节点（前端 + 后端一起启动）

```bash
source /opt/ros/noetic/setup.bash
source ~/leetcode/catkin_ws/devel/setup.bash
roslaunch msckf_vio msckf_vio_euroc.launch
```

> 此命令会同时启动 `image_processor`（前端）和 `vio`（后端）两个节点。  
> 看到 `Finish creating ROS IO...` 说明两个节点都已启动成功，等待数据输入。  
> **注意**：不要单独运行 `roslaunch msckf_vio image_processor_euroc.launch`，那样只有前端，没有位姿输出。

---

### 终端 3 — 启动 rviz 可视化

```bash
source /opt/ros/noetic/setup.bash
source ~/leetcode/catkin_ws/devel/setup.bash
rviz -d ~/leetcode/catkin_ws/src/msckf_vio/rviz/rviz_euroc_config.rviz
```

> rviz 窗口打开后会显示特征点追踪和轨迹估计结果。

#### rviz 无响应 / 无法打开的解决方案

**方案1：强制软件渲染（最稳定，优先尝试）**
```bash
source /opt/ros/noetic/setup.bash
source ~/leetcode/catkin_ws/devel/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_GL_VERSION_OVERRIDE=3.3
rviz -d ~/leetcode/catkin_ws/src/msckf_vio/rviz/rviz_euroc_config.rviz
```

**方案2：禁用 OGRE 阴影（减少渲染压力）**
```bash
source /opt/ros/noetic/setup.bash
source ~/leetcode/catkin_ws/devel/setup.bash
export OGRE_RTT_MODE=Copy
rviz -d ~/leetcode/catkin_ws/src/msckf_vio/rviz/rviz_euroc_config.rviz
```

**方案3：开启虚拟机 3D 加速（推荐从根本解决）**

- **VMware**：虚拟机设置 → 显示 → 勾选「加速 3D 图形」，显存设为 128MB 以上，重启虚拟机
- **VirtualBox**：设置 → 显示 → 勾选「启用 3D 加速」，显存设为 128MB，重启虚拟机

---

### 终端 4 — 播放 EuRoC 数据集

```bash
source /opt/ros/noetic/setup.bash
source ~/leetcode/catkin_ws/devel/setup.bash

# machine_hall数据集
rosbag play /home/dingxingyu/datasets/machine_hall/MH_05_difficult/MH_05_difficult.bag --clock

MH_02_easy
MH_03_medium
MH_04_difficult
MH_05_difficult


执行完执行这个:保存csv文件到本地

rostopic echo -p /firefly_sbx/vio/odom > ~/leetcode/catkin_ws/src/msckf_vio/datasets_output/machine_hall/MH_05_difficult/20260318/vio_odom.csv &
rostopic echo -p /firefly_sbx/image_processor/tracking_info > ~/leetcode/catkin_ws/src/msckf_vio/datasets_output/machine_hall/MH_05_difficult/20260318/tracking_info.csv &
rostopic echo -p /firefly_sbx/image_processor/features > ~/leetcode/catkin_ws/src/msckf_vio/datasets_output/machine_hall/MH_05_difficult/20260318/features.csv &
echo "所有话题录制已启动，PID 列表："
jobs -l

# vicon_room1数据集
rosbag play /home/dingxingyu/datasets/vicon_room1/V1_01_easy/V1_01_easy.bag --clock
rosbag play /home/dingxingyu/datasets/vicon_room1/V1_02_medium/V1_02_medium.bag --clock
rosbag play /home/dingxingyu/datasets/vicon_room1/V1_03_difficult/V1_03_difficult.bag --clock

# vicon_room2数据集
rosbag play /home/dingxingyu/datasets/vicon_room2/V2_01_easy/V2_01_easy.bag --clock
rosbag play /home/dingxingyu/datasets/vicon_room2/V2_02_medium/V2_02_medium.bag --clock
rosbag play /home/dingxingyu/datasets/vicon_room2/V2_03_difficult/V2_03_difficult.bag --clock

rostopic echo -p /firefly_sbx/vio/odom > ~/leetcode/catkin_ws/src/msckf_vio/datasets_output/vicon_room2/V2_01_easy/20260320/vio_odom.csv &
rostopic echo -p /firefly_sbx/image_processor/tracking_info > ~/leetcode/catkin_ws/src/msckf_vio/datasets_output//vicon_room2/V2_01_easy/20260320/tracking_info.csv &
rostopic echo -p /firefly_sbx/image_processor/features > ~/leetcode/catkin_ws/src/msckf_vio/datasets_output//vicon_room2/V2_01_easy/20260320/features.csv &
echo "所有话题录制已启动，PID 列表："
jobs -l


# bag 播放完成后，全部停止录制：
pkill -f "rostopic echo"

```
### 终端5：查看输出话题
> 播放开始后，终端2 会持续输出位姿估计信息，rviz 中可以看到实时轨迹。

---
#### 查看 msckf_vio 是否在发布位姿
source /opt/ros/noetic/setup.bash
source ~/leetcode/catkin_ws/devel/setup.bash

# 查看哪些节点在运行
rosnode list

# 查看所有正在发布的话题
rostopic list | grep firefly

# 查看 bag 播放进度（在终端4运行 rosbag play 后确认）
rostopic hz /imu0

# 查看 vio/odom 话题是否有数据
rostopic hz /firefly_sbx/vio/odom

/firefly_sbx/image_processor/debug_stereo_image	双目特征追踪可视化图像
/firefly_sbx/image_processor/features	提取的特征点
/firefly_sbx/vio/odom	VIO 估计的位姿（主输出）
/firefly_sbx/vio/feature_point_cloud	3D 路标点云
/firefly_sbx/vio/gt_odom	Ground Truth 位姿（用于对比）

# 极简打开rviz
pkill rviz
sleep 1
source /opt/ros/noetic/setup.bash
source ~/leetcode/catkin_ws/devel/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1
export OGRE_RTT_MODE=Copy
rviz -d ~/leetcode/catkin_ws/src/msckf_vio/rviz/rviz_euroc_lightweight.rviz


## 直接查看图像

```bash
source /opt/ros/noetic/setup.bash
source ~/leetcode/catkin_ws/devel/setup.bash
/opt/ros/noetic/bin/rosrun image_view image_view image:=/firefly_sbx/image_processor/debug_stereo_image
```

---

## 保存结果到 CSV 文件

在**终端4播放 bag 的同时**，另开一个终端运行以下命令，将所有话题实时写入 CSV：

```bash
source /opt/ros/noetic/setup.bash
source ~/leetcode/catkin_ws/devel/setup.bash

mkdir -p ~/leetcode/catkin_ws/src/msckf_vio/datasets_output/machine_hall/MH_01_easy/20260318
MH_02_easy
MH_03_medium
MH_04_difficult
MH_05_difficult

# 后台并行导出所有话题到各自的 CSV
    rostopic echo -p /firefly_sbx/vio/odom > ~/leetcode/catkin_ws/src/msckf_vio/datasets_output/machine_hall/MH_03_medium/20260318/vio_odom.csv &
    rostopic echo -p /firefly_sbx/image_processor/tracking_info > ~/leetcode/catkin_ws/src/msckf_vio/datasets_output/machine_hall/MH_03_medium/20260318/tracking_info.csv &
    rostopic echo -p /firefly_sbx/image_processor/features > ~/leetcode/catkin_ws/src/msckf_vio/datasets_output/machine_hall/MH_03_medium/20260318/features.csv &
    echo "所有话题录制已启动，PID 列表："
    jobs -l

# 以下两个话题仅在 Vicon Room 序列（V1_xx / V2_xx）有数据
# MH（Machine Hall）序列 ground truth 来自激光追踪仪，这两个话题无输出，可忽略
# rostopic echo -p /firefly_sbx/vio/gt_odom > ~/msckf_output/gt_odom.csv &
# rostopic echo -p /firefly_sbx/vio/mocap_odom > ~/msckf_output/mocap_odom.csv &

echo "所有话题录制已启动，PID 列表："
jobs -l
```

bag 播放完成后，全部停止录制：

```bash  
pkill -f "rostopic echo"
```

查看结果：

```bash
ls -lh ~/msckf_output/
head -3 ~/msckf_output/vio_odom.csv
```

> **注意**：`features` 和 `feature_point_cloud` 话题数据量很大，CSV 文件会快速增长。  
> 如果磁盘空间有限，可删去对应行，只保留位姿数据（`vio_odom.csv` 和 `gt_odom.csv`）。

---

## 可选：降速播放（计算资源不足时）

```bash
# 以 0.5 倍速播放，减轻 CPU 压力
rosbag play /home/dingxingyu/datasets/machine_hall/MH_01_easy/MH_01_easy.bag --clock --rate 0.5
```

---

## 其他可用数据集路径

```
/home/dingxingyu/datasets/machine_hall/MH_01_easy/MH_01_easy.bag
```

如需使用其他序列（V1_01_easy 等），将上面的路径替换即可，launch 文件无需修改。

---

## 验证安装是否正常（不启动数据集时）

```bash
source /opt/ros/noetic/setup.bash
source ~/leetcode/catkin_ws/devel/setup.bash

# 验证包路径
rospack find msckf_vio

# 验证 nodelet 插件注册
rospack plugins --attrib=plugin nodelet | grep msckf
```

正常输出：
```
/home/dingxingyu/leetcode/catkin_ws/src/msckf_vio
msckf_vio /home/dingxingyu/leetcode/catkin_ws/src/msckf_vio/nodelets.xml
```

---

## 常见问题

| 错误信息 | 解决方法 |
|---------|---------|
| `rospack: symbol lookup error` | `sudo ldconfig` 后重试 |
| `roscore` 已在运行，终端2报错 | 无需重启 roscore，直接运行终端2命令 |
| rviz 显示空白 | 确认终端4的 bag 正在播放，检查 Fixed Frame 是否设置为 `world` |
| `ERROR: cannot launch node of type [nodelet/nodelet]` | 重新执行 `source ~/leetcode/catkin_ws/devel/setup.bash` |
