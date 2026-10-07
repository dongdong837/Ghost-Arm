# Ghost Arm｜幽灵球机械臂仿真

A ROS 2 and Gazebo simulation of a floating robot with a 4-DOF robotic arm and gripper, featuring DH-based forward kinematics, analytical inverse kinematics, RGB-D sensing, body collision checks, and physics-based object grasping.

在球形飞行机器人后方安装四轴尾臂和双指夹爪，底部安装 RGB-D 相机，用于学习机械臂建模、正逆运动学、深度测量与物理抓取。项目重点是四轴机械臂的运动学计算、关节执行与抓取仿真。

## 已实现功能

- **机械结构与关节控制**：四个旋转关节、两个夹爪移动关节，SDF 物理模型与 URDF 显示模型。
- **正运动学**：标准 DH 参数建模，计算抓取中心相对球体的位置与姿态，并可对比 Gazebo 同时刻位姿。
- **解析逆运动学**：给定 XYZ 和末端俯仰角，求解双肘分支，检查关节限位并按当前姿态选解。
- **运动执行与球体避碰**：插值驱动关节，检查运动连杆与球体的间隙，拒绝穿过球体的直接关节路径。
- **RGB-D 测量**：底部彩色图、深度图精确时间戳配对，点击像素反投影为三维表面点，再转换到球体坐标系。
- **物理抓取**：夹取地面 4×4×6 厘米木块，依靠接触与摩擦抬起，默认末端俯仰角为 −60°。

## 环境与安装

已验证：Ubuntu 22.04、ROS 2 Humble、Gazebo Fortress（Ignition Gazebo 6）。先安装 ROS 2 Humble 并配置相应软件源，然后安装依赖：

```bash
sudo apt update
sudo apt install ros-humble-ros-gz ros-humble-rviz2 \
  ros-humble-robot-state-publisher ros-humble-cv-bridge \
  python3-numpy python3-opencv python3-yaml \
  build-essential cmake libignition-gazebo6-dev

git clone https://github.com/dongdong837/Ghost-Arm.git ghost_arm
cd ghost_arm
```

无需先执行 `colcon build`：Python 入口自动设置环境，启动脚本自动编译本项目的 50 Hz C++ 关节反馈插件。编译产物保存在 `runtime/build/`。

相机渲染需要可用的图形/OpenGL 上下文，即使不打开 Gazebo 窗口。在无硬件加速的虚拟机中，建议减少同时打开的可视化窗口。

## 快速启动

下面各终端都先进入克隆目录。

**终端 1：启动仿真后端，并打开 Gazebo 窗口。**

```bash
./ghost.sh start gui:=true
```

如果只使用 RViz 观察，可改用 `./ghost.sh start gui:=false`。Gazebo 后端仍负责物理与传感器仿真，RViz 只负责显示。

**终端 2：显示模型与传感器。**

```bash
./ghost.sh view
```

**终端 3：执行木块抓取。**

```bash
./ghost.sh grasp pick
# 松开木块，让其落地
./ghost.sh grasp release
```

请使用初始场景进行演示，操作期间不要同时运行遥控、其他关节命令或 Gazebo 关节滑块。默认 −60° 是抓取末端的总俯仰角，不是腕部单关节角度；可用 `./ghost.sh grasp pick --pitch-deg -30` 修改。

## 运动学与关节控制

```bash
# 预设姿态：前伸、下垂、闭合
./ghost.sh arm ready
./ghost.sh arm hang
./ghost.sh arm close

# 给定四个关节角，离线计算正运动学；不会驱动模型
./ghost.sh fk --joints 0 -0.8 -0.6 1.4
# 读取仿真关节角，并与 Gazebo 位姿比较
./ghost.sh fk --live

# 仅求逆解：XYZ 单位米，pitch 单位弧度
./ghost.sh ik 0 0 -0.46 --pitch -1.0471975512
# 求逆解，筛选球体避碰路径并实际运动
./ghost.sh ik 0 0 -0.46 --pitch -1.0471975512 --execute
```

运动学目标位于 `base_link` 球体坐标系，末端为 `grasp_center`。机械臂有四个自由度，只能指定位置与俯仰约束，不能任意指定完整六维位姿。纯数学逆解不检查碰撞；`--execute` 才执行球体间隙筛选。

## 底部 RGB-D 相机

```bash
./ghost.sh camera
# 或只测量指定像素
./ghost.sh camera --pixel 100 80
```

窗口左侧为 RGB、右侧为深度伪彩色。点击左侧图像可得到可见表面点在球体坐标系中的位置，不会触发机械臂运动。

**当前抓取演示使用 Gazebo 提供的木块位姿，尚未串联视觉识别、目标定位和自动抓取。** 深度测量与物理抓取目前是独立功能。

## 机械臂相关目录

```text
ghost_arm/
├── ghost.sh                    # 统一命令入口
├── src/
│   ├── gazebo_plugins/         # C++ 关节反馈插件
│   └── ghost_sim/
│       ├── simulation/         # 球体、尾臂、木块与场景生成
│       ├── kinematics/         # DH、正逆运动学、球体间隙检查
│       ├── control/            # 飞行、关节运动、抓取流程
│       ├── perception/         # RGB-D 同步、深度反投影
│       └── visualization/      # TF、模型与参考场景显示
├── assets/                     # 世界、模型与场景模板
├── config/                     # DH、桥接与 RViz 配置
├── launch/                     # ROS 2 启动文件
├── scripts/                    # 启动、编译、打包脚本
├── tests/                      # 单元测试与仿真集成检查
├── docs/                       # 中文教程与代码逐句讲解
├── runtime/                    # 自动生成，不上传
└── dist/                       # 源码包，不上传
```

以上列出机械臂相关的主要目录。代码由原 Ghost Sim 项目扩展，内部包名仍为 `ghost_sim`；仓库中保留的旧项目模块不作为本机械臂项目的功能介绍。项目与 Ghost Sim 共用 ROS_DOMAIN_ID=43 和 Gazebo 分区 `ghost_sim`，不要同时启动两套仿真。

## 测试与打包

```bash
./ghost.sh test
./ghost.sh package
```

源码包生成在 `dist/ghost-arm-source.tar.gz`。修改模型生成代码后，执行 `./ghost.sh build-model` 并重启仿真。

## 当前边界

- 机械臂路径检查针对连杆与球体，不是完整的环境碰撞规划，也没有接入 MoveIt。
- 世界采用零重力理想飞行，木块单独施加向下重力；不模拟旋翼或真实飞控。
- 抓取演示要求木块位于初始位置附近并平放，释放为张开后自由落下，不是任意物体抓取或精确放置。
- 虚拟机软件渲染可能低于实时速度；长期夹持悬停也可能缓慢下沉。

## 中文学习文档

- [Ghost Arm 代码逐句讲解](docs/Ghost_Arm代码逐句讲解.md)
- [DH 参数与正运动学](docs/DH参数与正运动学逐句讲解.md)
- [解析逆运动学](docs/逆运动学逐句讲解.md)
- [坐标逆解与运动执行](docs/坐标逆解与运动执行.md)
- [机械结构](docs/尾臂机械结构.md)
- [底部深度相机](docs/底部深度相机接入与逐句讲解.md)
- [木块抓取](docs/木块抓取操作与代码讲解.md)
- [倾斜抓取与相机性能修正](docs/倾斜抓取与相机卡顿修正.md)
- [目录说明](docs/目录说明.md)

文档中的 `/home/ubuntu/ghost_arm` 为开发机器路径，请替换为自己的克隆目录。标注为“早期”或原 Ghost Sim 的文档用于保留学习过程，当前入口与限制以本 README 和 Ghost Arm 主讲解为准。
