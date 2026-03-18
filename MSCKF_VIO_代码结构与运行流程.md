# MSCKF-VIO 代码结构与运行流程详解

> 基于论文 *"A Multi-State Constraint Kalman Filter for Vision-aided Inertial Navigation"* (Mourikis & Roumeliotis, 2007) 实现的双目视觉惯性里程计。

---

## 一、项目目录结构

```
msckf_vio/
├── CMakeLists.txt              # 编译配置
├── package.xml                 # ROS 包描述
├── nodelets.xml                # Nodelet 插件注册（ImageProcessorNodelet + MsckfVioNodelet）
│
├── include/msckf_vio/          # 头文件
│   ├── imu_state.h             # IMU 状态结构体 (IMUState)
│   ├── cam_state.h             # 相机状态结构体 (CAMState)
│   ├── feature.hpp             # 特征点结构体与三角化 (Feature)
│   ├── image_processor.h       # 前端：特征提取与追踪类 (ImageProcessor)
│   ├── image_processor_nodelet.h  # ImageProcessor 的 Nodelet 封装
│   ├── msckf_vio.h             # 后端：MSCKF 滤波器核心类 (MsckfVio)
│   ├── msckf_vio_nodelet.h     # MsckfVio 的 Nodelet 封装
│   ├── math_utils.hpp          # 数学工具（四元数、旋转、斜对称矩阵等）
│   └── utils.h                 # 通用工具函数
│
├── src/                        # 源文件
│   ├── image_processor.cpp     # 前端实现（~1500 行）
│   ├── image_processor_nodelet.cpp  # ImageProcessor Nodelet 入口
│   ├── msckf_vio.cpp           # 后端实现（~1450 行）
│   ├── msckf_vio_nodelet.cpp   # MsckfVio Nodelet 入口
│   └── utils.cpp               # 工具函数实现
│
├── msg/                        # 自定义 ROS 消息
│   ├── FeatureMeasurement.msg  # 单个特征：id + cam0/cam1 归一化坐标 (u0,v0,u1,v1)
│   ├── CameraMeasurement.msg   # 一帧所有特征：Header + FeatureMeasurement[]
│   └── TrackingInfo.msg        # 追踪统计：before/after_tracking/matching/ransac
│
├── config/                     # 相机-IMU 标定参数 (YAML)
│   ├── camchain-imucam-euroc.yaml
│   └── ...
│
├── launch/                     # ROS 启动文件
│   ├── image_processor_euroc.launch   # 前端节点启动
│   ├── msckf_vio_euroc.launch         # 完整系统启动（含前端+后端）
│   └── ...
│
├── rviz/                       # rviz 可视化配置
├── test/                       # 单元测试
├── analysis/                   # Python 分析脚本
└── datasets_output/            # 运行结果 CSV
```

---

## 二、系统架构总览

系统分为 **前端（ImageProcessor）** 和 **后端（MsckfVio）** 两个 ROS Nodelet，通过话题通信：

```
                    ┌─────────────────────────────────────────────────────┐
                    │                     ROS 话题                        │
                    └─────────────────────────────────────────────────────┘
                                            │
         ┌──────────────────────────────────┼──────────────────────────────┐
         │                                  │                              │
    /cam0/image_raw                    /imu0                     /cam1/image_raw
    /cam1/image_raw                      │                              │
         │                               │                              │
         ▼                               ▼                              │
┌─────────────────────────┐    ┌─────────────────────────┐              │
│   ImageProcessorNodelet │    │     MsckfVioNodelet      │              │
│   ┌───────────────────┐ │    │   ┌───────────────────┐  │              │
│   │  ImageProcessor   │ │    │   │    MsckfVio       │  │              │
│   │                   │ │    │   │                   │  │              │
│   │ • 特征检测 (FAST) │ │    │   │ • IMU 预积分      │  │              │
│   │ • 光流追踪 (KLT)  │ │    │   │ • 状态增广        │  │              │
│   │ • 双目匹配        │ │    │   │ • EKF 量测更新    │  │              │
│   │ • 2-point RANSAC  │ │    │   │ • 滑动窗口管理    │  │              │
│   │ • 网格管理        │ │    │   │ • 位姿发布        │  │              │
│   └───────┬───────────┘ │    │   └───────────────────┘  │              │
│           │              │    │            ▲              │              │
└───────────┼──────────────┘    └────────────┼──────────────┘              │
            │                                │                              │
            │    features (CameraMeasurement) │                              │
            └────────────────────────────────┘                              │
                                                                            │
            ┌───────────────── 输出话题 ──────────────────┐                 │
            │                                              │                │
            ▼                                              ▼                │
    /firefly_sbx/vio/odom                   /firefly_sbx/vio/              │
    (nav_msgs/Odometry)                     feature_point_cloud            │
                                            (sensor_msgs/PointCloud2)      │
```

---

## 三、核心数据结构

### 3.1 IMUState（`imu_state.h`）

IMU 状态向量，维度 = **21**（误差状态 15 + 外参 6）：

| 成员变量 | 类型 | 含义 |
|---------|------|------|
| `id` | `StateIDType (long long)` | 唯一标识符 |
| `time` | `double` | 时间戳 |
| `orientation` | `Vector4d` | 四元数 $q_{WI}$（world→IMU） |
| `position` | `Vector3d` | 世界系下 IMU 位置 |
| `velocity` | `Vector3d` | 世界系下 IMU 速度 |
| `gyro_bias` | `Vector3d` | 陀螺仪零偏 |
| `acc_bias` | `Vector3d` | 加速度计零偏 |
| `R_imu_cam0` | `Matrix3d` | IMU→cam0 旋转 |
| `t_cam0_imu` | `Vector3d` | IMU→cam0 平移 |
| `orientation_null` | `Vector4d` | 可观性约束用 |
| `position_null` | `Vector3d` | 可观性约束用 |
| `velocity_null` | `Vector3d` | 可观性约束用 |

**静态成员**：`gyro_noise`, `acc_noise`, `gyro_bias_noise`, `acc_bias_noise`, `gravity`, `T_imu_body`

### 3.2 CAMState（`cam_state.h`）

滑动窗口中的相机状态，维度 = **6**（3 旋转 + 3 平移）：

| 成员变量 | 类型 | 含义 |
|---------|------|------|
| `id` | `StateIDType` | 对应 IMU 状态 ID |
| `time` | `double` | 时间戳 |
| `orientation` | `Vector4d` | 四元数 $q_{WC}$（world→camera） |
| `position` | `Vector3d` | 世界系下相机位置 |
| `orientation_null` | `Vector4d` | 可观性约束用 |
| `position_null` | `Vector3d` | 可观性约束用 |

**静态成员**：`T_cam0_cam1`（左右目外参）

**`CamStateServer`** = `std::map<StateIDType, CAMState>`：滑动窗口中所有相机状态

### 3.3 Feature（`feature.hpp`）

特征点数据结构，包含三角化逻辑：

| 成员/方法 | 含义 |
|---------|------|
| `id` | 特征唯一 ID |
| `observations` | `map<StateIDType, Vector4d>`：该特征在各相机状态下的观测 (u0,v0,u1,v1) |
| `position` | 三角化后的 3D 位置（世界系） |
| `is_initialized` | 是否已三角化 |
| `initializePosition()` | 使用 SVD 最小二乘法三角化特征 3D 位置 |
| `optimization_config` | 三角化优化配置（LM 最大迭代/精度/translation_threshold） |

### 3.4 StateServer（`msckf_vio.h` 内嵌）

```cpp
struct StateServer {
    IMUState imu_state;                              // 当前 IMU 状态
    CamStateServer cam_states;                       // 滑动窗口相机状态
    Eigen::MatrixXd state_cov;                       // 状态协方差矩阵 P
    Eigen::Matrix<double, 12, 12> continuous_noise_cov;  // 连续噪声协方差 Q_c
};
```

### 3.5 MapServer

```cpp
typedef std::map<FeatureIDType, Feature> MapServer;  // 特征 ID → Feature
```

---

## 四、前端：ImageProcessor 详细流程

### 4.1 初始化流程

```
ImageProcessorNodelet::onInit()
    └── ImageProcessor::initialize()
            ├── loadParameters()     // 从 ROS 参数服务器加载相机内外参、网格参数、光流参数等
            └── createRosIO()        // 创建订阅/发布
                    ├── 订阅: ~cam0_image, ~cam1_image (TimeSynchronizer 同步双目)
                    ├── 订阅: ~imu
                    ├── 发布: ~features          (CameraMeasurement)
                    ├── 发布: ~tracking_info      (TrackingInfo)
                    └── 发布: ~debug_stereo_image (Image)
```

### 4.2 IMU 回调

```cpp
void imuCallback(const sensor_msgs::ImuConstPtr& msg)
    // 缓存 IMU 数据到 imu_msg_buffer，供帧间旋转预测使用
```

### 4.3 双目图像回调（核心入口）

```
stereoCallback(cam0_img, cam1_img)
│
├── 1. cv_bridge::toCvShare()         // 获取灰度图像
├── 2. createImagePyramids()          // 构建光流金字塔（cam0 + cam1）
│
├── [首帧] initializeFirstFrame()
│       ├── FAST 特征检测（cam0）
│       ├── undistortPoints()          // 去畸变+归一化
│       ├── stereoMatch()              // 在 cam1 中匹配（光流追踪）
│       ├── 分配到网格 grid
│       └── 裁剪每格特征数 ≤ grid_max_feature_num
│
├── [后续帧] 
│   ├── 3. trackFeatures()            // 光流追踪 + 外点剔除
│   │       ├── integrateImuData()     // 积分 IMU 获取帧间旋转 R_p_c
│   │       ├── predictFeatureTracking() // 用旋转预测特征位置（减少光流搜索范围）
│   │       ├── calcOpticalFlowPyrLK() // cam0 上一帧→当前帧 光流追踪
│   │       ├── 反向光流验证            // 当前帧→上一帧，检查往返误差
│   │       ├── stereoMatch()          // 在 cam1 当前帧中匹配
│   │       ├── twoPointRansac()       // 2-point RANSAC 剔除 cam0 外点
│   │       ├── twoPointRansac()       // 2-point RANSAC 剔除 cam1 外点
│   │       ├── undistortPoints()      // 去畸变
│   │       └── 更新追踪统计 → 发布 TrackingInfo
│   │
│   ├── 4. addNewFeatures()           // 检测新特征补充
│   │       ├── 构建已有特征 mask（90 像素抑制半径）
│   │       ├── FAST 检测（cam0 当前帧）
│   │       ├── stereoMatch()          // 双目匹配新特征
│   │       └── 分配到网格
│   │
│   └── 5. pruneGridFeatures()        // 每格保留生命周期最长的 grid_max_feature_num 个特征
│
├── 6. drawFeaturesStereo()           // 绘制调试图像
│
├── 7. publish()                      // 发布 CameraMeasurement 消息
│       └── 将所有网格中的特征(id, u0, v0, u1, v1)打包发布
│
└── 8. 更新: prev ← curr, 清空 curr_features
```

### 4.4 关键算法

#### twoPointRansac（2-Point RANSAC）

使用 2 点模型（基于 IMU 旋转补偿后的本质矩阵约束）剔除外点：
1. 根据帧间 IMU 旋转将前一帧特征转换到当前帧
2. 随机采样 2 对匹配点，求解平移方向 $t$
3. 构建本质矩阵 $E = [t]_\times R$
4. 用对极约束检验所有匹配
5. 多次迭代选最优内点集

---

## 五、后端：MsckfVio 详细流程

### 5.1 初始化流程

```
MsckfVioNodelet::onInit()
    └── MsckfVio::initialize()
            ├── loadParameters()      // 加载噪声参数、阈值、外参等
            │       ├── IMUState 噪声: gyro_noise, acc_noise, gyro_bias_noise, acc_bias_noise
            │       ├── 外参: T_imu_cam0, T_cn_cnm1
            │       ├── 滤波器参数: max_cam_state_size, position_std_threshold
            │       ├── 关键帧阈值: rotation_threshold, translation_threshold, tracking_rate_threshold
            │       └── 初始速度、初始协方差
            │
            ├── 初始化 state_server
            │       ├── state_cov (21×21 初始协方差矩阵)
            │       └── continuous_noise_cov (12×12 连续噪声)
            │
            ├── 初始化 chi_squared_test_table (门限检验表)
            │
            └── createRosIO()
                    ├── 订阅: ~imu          → imuCallback
                    ├── 订阅: ~features      → featureCallback
                    ├── 订阅: ~mocap_odom    → mocapOdomCallback (调试用)
                    ├── 发布: ~odom                 (nav_msgs/Odometry)
                    ├── 发布: ~feature_point_cloud  (sensor_msgs/PointCloud2)
                    ├── 发布: ~path, ~gt_odom, ~mocap_odom
                    └── 服务: ~reset         → resetCallback
```

### 5.2 IMU 回调

```cpp
void imuCallback(const sensor_msgs::ImuConstPtr& msg)
    ├── 缓存到 imu_msg_buffer
    └── [首次收到200条] initializeGravityAndBias()
            ├── 计算前 N 帧 IMU 均值 → 初始 gyro_bias
            ├── 用加速度方向确定初始姿态（使重力对齐 z 轴）
            ├── gravity = [0, 0, -GRAVITY_ACCELERATION]
            └── is_gravity_set = true
```

### 5.3 特征回调（核心主循环）

收到前端发来的 `CameraMeasurement` 触发一次完整的 EKF 更新：

```
featureCallback(msg)
│
├── [guard] 重力未初始化 → return
├── [首帧] 记录起始时间
│
├── 1. batchImuProcessing(msg.time)  ──────────── IMU 状态传播
│       └── 遍历 imu_msg_buffer 中时间 ≤ msg.time 的 IMU 数据
│           └── processModel(time, gyro, acc)     // 逐条处理
│                   ├── (a) 计算连续时间状态转移矩阵 F (21×21)
│                   │       ├── F_11: 姿态项 (-[ω]×)
│                   │       ├── F_13: 陀螺偏差项
│                   │       ├── F_21: 加速度→速度
│                   │       ├── F_25: 加计偏差→速度
│                   │       └── F_31: 速度→位置
│                   │
│                   ├── (b) 计算噪声映射矩阵 G (21×12)
│                   │
│                   ├── (c) 离散化状态转移: Φ = I + F·dt + F²·dt²/2 + ...
│                   │
│                   ├── (d) [OC-EKF修正] 使用 null-space 修改 Φ
│                   │       使可观性矩阵具有正确的零空间
│                   │
│                   ├── (e) 预测新状态: predictNewState(dt, gyro, acc)
│                   │       ├── 四元数 4 阶 Runge-Kutta 积分
│                   │       ├── 速度：v += (R·a + g)·dt
│                   │       └── 位置：p += v·dt + 0.5·(R·a + g)·dt²
│                   │
│                   └── (f) 传播协方差:
│                           P_II = Φ · P_II · Φᵀ + Φ · G · Q · Gᵀ · Φᵀ · dt
│                           P_IC = Φ · P_IC
│                           对称化: P = (P + Pᵀ)/2
│
├── 2. stateAugmentation(msg.time)  ──────────── 状态增广
│       ├── 计算新相机位姿:
│       │       q_cam = q_imu ⊗ q_imu_cam
│       │       p_cam = p_imu + R_imu · t_cam_imu
│       │
│       ├── 构建增广雅可比 J (6×21):
│       │       关联新相机状态与当前 IMU 状态的关系
│       │
│       └── 扩展协方差矩阵:
│               P_new = [  I       ] · P · [ I  Jᵀ ]
│                       [  J       ]       [       ]
│               (矩阵维度从 N → N+6)
│
├── 3. addFeatureObservations(msg)  ──────────── 添加特征观测
│       ├── 遍历 msg 中所有 FeatureMeasurement
│       ├── 新特征 → 添加到 map_server
│       ├── 已有特征 → 追加观测 (cam_state_id, [u0,v0,u1,v1])
│       └── 计算 tracking_rate = tracked / total
│
├── 4. removeLostFeatures()  ────────────────── 移除丢失特征 + EKF 更新
│       ├── 遍历 map_server 中所有特征
│       ├── 筛选"丢失特征"（不在最新相机状态中观测到）
│       │   和"追踪过长的特征"（观测数 == max_cam_state_size）
│       │
│       ├── 对每个丢失特征：
│       │   ├── 检查观测次数 ≥ 3（否则丢弃）
│       │   ├── Feature::initializePosition() ── 三角化 3D 位置
│       │   │       ├── 选择首/尾两个相机状态
│       │   │       ├── SVD 线性三角化初始值
│       │   │       └── (可选) Levenberg-Marquardt 非线性优化
│       │   │
│       │   ├── featureJacobian()
│       │   │       ├── 对该特征的所有观测调用 measurementJacobian()
│       │   │       │       ├── 计算单次观测的 H_x (4×6N) 和 H_f (4×3)
│       │   │       │       └── 计算残差 r (4×1)
│       │   │       ├── 堆叠所有观测的雅可比
│       │   │       ├── 左零空间投影：消除特征位置 H_f
│       │   │       │       H_o = A_left_null_space' · H_x
│       │   │       │       r_o = A_left_null_space' · r
│       │   │       └── [OC-EKF修正] 使用 null 姿态修改雅可比
│       │   │
│       │   └── gatingTest()  ── 卡方门限检验
│       │           r'·(H·P·H'+σ²I)⁻¹·r < χ²(dof)
│       │
│       ├── 堆叠所有通过检验的特征的 H_x, r_o
│       │
│       └── measurementUpdate(H, r)  ──── EKF 量测更新
│               ├── [若 H 行数 > 列数] QR 分解压缩:
│               │       H = [Q₁ Q₂] · [T_H; 0]
│               │       只保留 T_H 和对应 r 部分
│               │
│               ├── 卡尔曼增益: K = P·Hᵀ·(H·P·Hᵀ + σ²I)⁻¹
│               ├── 状态修正: δx = K · r
│               │       ├── IMU 姿态: q ← q ⊗ δθ
│               │       ├── IMU gyro_bias += δb_g
│               │       ├── IMU velocity += δv
│               │       ├── IMU acc_bias += δb_a
│               │       ├── IMU position += δp
│               │       ├── IMU 外参 R_imu_cam0 修正
│               │       ├── IMU 外参 t_cam0_imu += δt
│               │       └── 各 CAMState 姿态、位置修正
│               │
│               └── 协方差更新: P = (I - K·H) · P · (I - K·H)ᵀ + K·σ²·Kᵀ
│                   (Joseph 形式，保证正定)
│
├── 5. pruneCamStateBuffer()  ───────────────── 裁剪相机状态滑动窗口
│       ├── findRedundantCamStates()
│       │       ├── 关键帧判定：与最新帧比较
│       │       │   ├── 旋转差 < rotation_threshold
│       │       │   ├── 平移差 < translation_threshold
│       │       │   └── tracking_rate > tracking_rate_threshold
│       │       │
│       │       └── 非关键帧：移除最近的冗余帧
│       │           关键帧：移除最旧的帧（保留最新 + 最旧两端）
│       │
│       ├── 对关联到待移除相机状态的特征执行量测更新
│       │   （同 removeLostFeatures 的三角化+雅可比+EKF更新流程）
│       │
│       └── 从 state_cov 和 cam_states 中移除对应行列
│
├── 6. publish(msg.time)  ───────────────────── 发布结果
│       ├── 将 IMU 位姿转换到 body 系: T_body = T_imu_body · T_world_imu
│       ├── 计算 body 系速度
│       ├── 发布 nav_msgs/Odometry（含位姿协方差 6×6 + 速度协方差 3×3）
│       ├── 发布 TF: world → odom
│       └── 发布 feature_point_cloud (所有已三角化特征的 3D 位置)
│
└── 7. onlineReset()  ───────────────────────── 在线重置检查
        └── 若位置标准差 > position_std_threshold → 重置滤波器状态
```

---

## 六、状态向量与协方差矩阵结构

### 6.1 完整状态向量

$$
\mathbf{x} = \begin{bmatrix} \mathbf{x}_{IMU} \\ \mathbf{x}_{C_1} \\ \mathbf{x}_{C_2} \\ \vdots \\ \mathbf{x}_{C_N} \end{bmatrix}
$$

其中：
- $\mathbf{x}_{IMU}$ = $[\delta\boldsymbol{\theta}_{3\times1}, \delta\mathbf{b}_g, \delta\mathbf{v}, \delta\mathbf{b}_a, \delta\mathbf{p}, \delta\boldsymbol{\theta}_{ext}, \delta\mathbf{t}_{ext}]$ — **21** 维误差状态
- $\mathbf{x}_{C_i}$ = $[\delta\boldsymbol{\theta}_{C_i}, \delta\mathbf{p}_{C_i}]$ — **6** 维相机误差状态

### 6.2 协方差矩阵

$$
\mathbf{P} = \begin{bmatrix} \mathbf{P}_{II} & \mathbf{P}_{IC} \\ \mathbf{P}_{CI} & \mathbf{P}_{CC} \end{bmatrix}
$$

- 维度: $(21 + 6N) \times (21 + 6N)$，N = 滑动窗口中相机状态数（最大 `max_cam_state_size=20`）
- 最大: $141 \times 141$

---

## 七、关键算法详解

### 7.1 IMU 运动模型（processModel + predictNewState）

**连续时间模型**：

$$\dot{\mathbf{q}} = \frac{1}{2} \boldsymbol{\Omega}(\boldsymbol{\omega}) \mathbf{q}$$

$$\dot{\mathbf{v}} = \mathbf{R}^T \mathbf{a} + \mathbf{g}$$

$$\dot{\mathbf{p}} = \mathbf{v}$$

**状态积分**（predictNewState）：使用 4 阶 Runge-Kutta 方法积分四元数。

**协方差传播**：

$$\mathbf{P}_{k+1} = \boldsymbol{\Phi}_k \mathbf{P}_k \boldsymbol{\Phi}_k^T + \boldsymbol{\Phi}_k \mathbf{G}_k \mathbf{Q} \mathbf{G}_k^T \boldsymbol{\Phi}_k^T \cdot \Delta t$$

### 7.2 OC-EKF 可观性约束

为保证 MSCKF 具有正确的不可观子空间（4 维：全局旋转 + 全局平移的一个方向），代码在以下位置做了修正：
- `processModel()`：使用 `null` 姿态修改状态转移矩阵 $\Phi$
- `featureJacobian()`：使用 `null` 姿态计算量测雅可比 $H$

### 7.3 量测模型（measurementJacobian）

对于特征 $j$ 在相机状态 $i$ 下的观测：

$$\mathbf{z}_{ij} = \begin{bmatrix} u_0 \\ v_0 \\ u_1 \\ v_1 \end{bmatrix} = \pi(\mathbf{p}_j^{C_i^0}) \oplus \pi(\mathbf{T}_{C_1C_0} \cdot \mathbf{p}_j^{C_i^0})$$

雅可比分解为：
- $H_x$：对相机状态的雅可比（4×6）
- $H_f$：对特征位置的雅可比（4×3）

通过**左零空间投影** $H_f$ 被消除，仅保留 $H_x$。

### 7.4 QR 分解压缩

当量测维度远大于状态维度时：

$$\mathbf{H} = \begin{bmatrix} \mathbf{Q}_1 & \mathbf{Q}_2 \end{bmatrix} \begin{bmatrix} \mathbf{T}_H \\ \mathbf{0} \end{bmatrix}$$

只用 $\mathbf{T}_H$ 和 $\mathbf{Q}_1^T \mathbf{r}$ 做更新，大幅降低计算量。

### 7.5 特征三角化（Feature::initializePosition）

使用 SVD 线性三角化 + 可选 LM 非线性优化：
1. 选择具有足够基线的两个相机视角
2. 在归一化平面上构建 DLT 方程
3. SVD 求解最小特征值对应的 3D 位置
4. 检查：深度 > 0、重投影误差 < 阈值

---

## 八、ROS 话题与参数

### 8.1 话题（EuRoC 配置，namespace = `firefly_sbx`）

| 话题名 | 类型 | 方向 | 说明 |
|--------|------|------|------|
| `/imu0` | `sensor_msgs/Imu` | 输入 | IMU 数据（200Hz） |
| `/cam0/image_raw` | `sensor_msgs/Image` | 输入 | 左目图像（20Hz） |
| `/cam1/image_raw` | `sensor_msgs/Image` | 输入 | 右目图像（20Hz） |
| `image_processor/features` | `CameraMeasurement` | 内部 | 前端→后端特征传递 |
| `image_processor/tracking_info` | `TrackingInfo` | 输出 | 追踪统计 |
| `image_processor/debug_stereo_image` | `Image` | 输出 | 调试图像 |
| `vio/odom` | `nav_msgs/Odometry` | 输出 | **VIO 位姿（主输出）** |
| `vio/feature_point_cloud` | `PointCloud2` | 输出 | 3D 路标点云 |

### 8.2 关键参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `frame_rate` | 20 | 图像帧率 |
| `max_cam_state_size` | 20 | 滑动窗口最大相机状态数 |
| `position_std_threshold` | 8.0 | 在线重置阈值 |
| `noise/gyro` | 0.005 | 陀螺噪声标准差 |
| `noise/acc` | 0.05 | 加速度计噪声标准差 |
| `noise/feature` | 0.035 | 特征观测噪声标准差 |
| `grid_row × grid_col` | 4×5 | 特征网格划分 |
| `grid_max_feature_num` | 4 | 每格最大特征数 |
| `ransac_threshold` | 3 | RANSAC 像素阈值 |

---

## 九、推荐阅读顺序

```
第 1 步：理解数据结构
    imu_state.h → cam_state.h → feature.hpp → math_utils.hpp

第 2 步：理解前端（视觉特征处理）
    image_processor.h → image_processor.cpp
    重点函数：stereoCallback → trackFeatures → twoPointRansac → publish

第 3 步：理解后端（MSCKF 滤波器）
    msckf_vio.h → msckf_vio.cpp
    重点函数：featureCallback (主循环)
             → batchImuProcessing → processModel → predictNewState
             → stateAugmentation
             → removeLostFeatures → featureJacobian → measurementUpdate
             → pruneCamStateBuffer
             → publish

第 4 步：理解系统启动
    msckf_vio_euroc.launch → image_processor_euroc.launch
    msckf_vio_nodelet.cpp → image_processor_nodelet.cpp

第 5 步：配置与标定
    config/camchain-imucam-euroc.yaml
```

---

## 十、代码行数统计

| 文件 | 行数（约） | 核心内容 |
|------|-----------|---------|
| `msckf_vio.cpp` | ~1450 | 后端 EKF 核心 |
| `image_processor.cpp` | ~1500 | 前端特征处理 |
| `feature.hpp` | ~450 | 特征三角化 |
| `msckf_vio.h` | ~240 | 后端类定义 |
| `image_processor.h` | ~250 | 前端类定义 |
| `imu_state.h` | ~110 | IMU 状态 |
| `cam_state.h` | ~70 | 相机状态 |
| `math_utils.hpp` | ~200 | 四元数/旋转工具 |
| `utils.cpp` | ~30 | IMU数据排序 |
| 两个 `*_nodelet.cpp` | ~25 each | Nodelet 入口 |

**总核心代码量约 4300 行。**
