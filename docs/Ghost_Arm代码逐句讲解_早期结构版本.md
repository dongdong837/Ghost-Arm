> 历史快照：此文件只保留早期结构与手动控制阶段的讲解。当前源码、默认参数和操作见 [当前逐句讲解](Ghost_Arm代码逐句讲解.md)。

# Ghost Arm 代码逐句讲解

本文件对应当前机械结构与手动关节控制版本。阅读顺序：结构参数 → 模型生成 → 姿态控制 → ROS/Gazebo 桥 → 启动与界面。

## 0. 两个项目分别在哪里

- `/home/ubuntu/ghost_sim`：保留原来的幽灵球、相机、三维导航与大模型项目。
- `/home/ubuntu/ghost_arm`：新增尾臂、夹爪、底部相机和 Gazebo 控制面板的项目。

两个目录各自保存文件，修改新项目不会修改原项目。内部 Python 包仍叫 `ghost_sim`，这只是导入名称，不表示程序必须放在旧目录。当前共用 ROS 域和 Gazebo 分区，因此不要同时运行两个项目。

原有飞行、三维导航和大模型代码见 [原项目代码讲解](代码逐句讲解.md)。本篇把新增机械臂代码展开解释，没有把尚未实现的 DH、逆运动学、视觉抓取写成已经具备的功能。

## 1. 先理解数据如何流动

```text
Gazebo 关节面板 ──Gazebo Double 消息────────────┐
                                               ↓
ghost.sh arm ready → Python → ROS Float64 → 桥接 → Gazebo 关节位置控制器
                                               ↓
                                       物理引擎计算关节运动
                                               ↓
                  JointStatePublisher → 桥接 → /joint_states
                                               ↓
                           姿态完成检查 / robot_state_publisher → TF → RViz
```

GUI 滑块直接走 Gazebo 通信；终端命令经过 ROS 桥。两条路径控制同一组关节，不要同时操作。

- `link`：刚性连杆；`joint`：连杆之间的连接与运动约束。
- `visual`：看到的外形；`collision`：物理碰撞形状。画出一个零件不等于它能够参与碰撞。
- `fixed`：固定连接；`revolute`：有角度限位的旋转关节；`prismatic`：直线滑动关节。
- 位置、尺寸用米，质量用千克，旋转角用弧度。1 rad 约为 57.3°。
- 当前 +X 为幽灵前方，+Y 为左方，+Z 为上方；尾臂从 -X 侧伸出。

下文代码块为源文件快照，表格行号包含空行。空行只分隔段落，不执行操作；同一行中用分号连接的语句分别解释。

## 2. 定义尾臂结构：tail_structure.py

源文件：`src/ghost_sim/simulation/tail_structure.py`

```python
"""Shared mechanical geometry for Gazebo and ROS; metres, radians, +X forward."""
import xml.etree.ElementTree as E

JOINTS = ['tail_yaw', 'tail_shoulder', 'tail_elbow', 'tail_wrist', 'finger_left_slide', 'finger_right_slide']
# parent, child, joint, type, joint origin in parent, axis, limits, box size, box centre, mass
PARTS = [
 ('base_link','bottom_mount','bottom_mount_fixed','fixed',(0,0,-.25),(0,0,1),(0,0),(.24,.18,.035),(0,0,0),.08),
 ('bottom_mount','grasp_camera_link','grasp_camera_fixed','fixed',(.065,0,-.04),(0,0,1),(0,0),(.065,.10,.035),(0,0,0),.04),
 ('base_link','tail_bracket','tail_bracket_fixed','fixed',(-.27,0,-.04),(0,0,1),(0,0),(.14,.09,.07),(-.025,0,0),.06),
 ('tail_bracket','tail_yaw_link','tail_yaw','revolute',(-.06,0,-.04),(0,0,1),(-1.3,1.3),(.07,.085,.055),(0,0,-.012),.05),
 ('tail_yaw_link','tail_upper','tail_shoulder','revolute',(0,0,-.045),(0,1,0),(-1.6,.7),(.045,.055,.22),(0,0,-.11),.08),
 ('tail_upper','tail_lower','tail_elbow','revolute',(0,0,-.22),(0,1,0),(-1.8,1.8),(.04,.05,.20),(0,0,-.10),.06),
 ('tail_lower','gripper_palm','tail_wrist','revolute',(0,0,-.20),(0,1,0),(-2.2,2.2),(.065,.115,.055),(0,0,-.035),.06),
 ('gripper_palm','finger_left','finger_left_slide','prismatic',(0,.018,-.06),(0,1,0),(0,.03),(.045,.016,.085),(0,0,-.0425),.02),
 ('gripper_palm','finger_right','finger_right_slide','prismatic',(0,-.018,-.06),(0,-1,0),(0,.03),(.045,.016,.085),(0,0,-.0425),.02),
 ('gripper_palm','grasp_center','grasp_center_fixed','fixed',(0,0,-.115),(0,0,1),(0,0),None,(0,0,0),.001),
]

def add(p,t,v=None,**kw):
 e=E.SubElement(p,t,kw)
 if v is not None:e.text=str(v)
 return e

def vec(v):return ' '.join(map(str,v))

def attach(sdf_model, robot):
 # Fortress scene serialization cannot handle relative_to poses reliably.
 # All zero-pose joint origins have identity rotation, so sum translations.
 zero_positions={'base_link':(0,0,0)}
 for parent,child,name,kind,origin,axis,limits,size,centre,mass in PARTS:
  sl=add(sdf_model,'link',name=child)
  zero_positions[child]=tuple(a+b for a,b in zip(zero_positions[parent],origin))
  add(sl,'pose',vec(zero_positions[child])+' 0 0 0')
  inertia=add(sl,'inertial');add(inertia,'pose',vec(centre)+' 0 0 0');add(inertia,'mass',mass)
  tensor=add(inertia,'inertia');a,b,c=size or (.01,.01,.01)
  for key,value in [('ixx',mass*(b*b+c*c)/12),('iyy',mass*(a*a+c*c)/12),('izz',mass*(a*a+b*b)/12),('ixy',0),('ixz',0),('iyz',0)]:add(tensor,key,value)
  ul=add(robot,'link',name=child)
  if size:
   color='0.10 0.20 0.28 1' if 'finger' in child or 'camera' in child else '0.15 0.65 0.8 1'
   for kind_geom in ('visual','collision'):
    sg=add(sl,kind_geom,name=child+'_'+kind_geom);add(sg,'pose',vec(centre)+' 0 0 0')
    add(add(add(sg,'geometry'),'box'),'size',vec(size))
    ug=add(ul,kind_geom);add(ug,'origin',xyz=vec(centre),rpy='0 0 0');add(add(ug,'geometry'),'box',size=vec(size))
    if kind_geom=='visual':
     mat=add(sg,'material');add(mat,'ambient',color);add(mat,'diffuse',color)
     add(add(ug,'material',name=child+'_color'),'color',rgba=color)
   # Visible bearing aligned with the pitch joint axis.
   if kind=='revolute':
    bearing=add(sl,'visual',name='bearing');add(bearing,'pose','0 0 0 1.57079632679 0 0' if axis==(0,1,0) else '0 0 0 0 0 0')
    cyl=add(add(bearing,'geometry'),'cylinder');add(cyl,'radius',.032);add(cyl,'length',.075)
    mat=add(bearing,'material');add(mat,'ambient','0.85 0.55 0.12 1');add(mat,'diffuse','0.85 0.55 0.12 1')
    uv=add(ul,'visual');add(uv,'origin',xyz='0 0 0',rpy='1.57079632679 0 0' if axis==(0,1,0) else '0 0 0');add(add(uv,'geometry'),'cylinder',radius='.032',length='.075');add(add(uv,'material',name='bearing_gold'),'color',rgba='0.85 0.55 0.12 1')
  sj=add(sdf_model,'joint',name=name,type=kind);add(sj,'parent',parent);add(sj,'child',child)
  uj=add(robot,'joint',name=name,type=kind);add(uj,'parent',link=parent);add(uj,'child',link=child);add(uj,'origin',xyz=vec(origin),rpy='0 0 0')
  if kind!='fixed':
   ax=add(sj,'axis');add(ax,'xyz',vec(axis));limit=add(ax,'limit')
   for key,val in [('lower',limits[0]),('upper',limits[1]),('effort',8 if kind=='revolute' else 3),('velocity',1 if kind=='revolute' else .06)]:add(limit,key,val)
   add(add(ax,'dynamics'),'damping',.08)
   add(uj,'axis',xyz=vec(axis));add(uj,'limit',lower=str(limits[0]),upper=str(limits[1]),effort='8',velocity='1' if kind=='revolute' else '.06')
   controller=add(sdf_model,'plugin',filename='ignition-gazebo-joint-position-controller-system',name='ignition::gazebo::systems::JointPositionController')
   for key,val in [('joint_name',name),('topic','/model/ghost/joint/'+name+'/0/cmd_pos'),('p_gain',5 if kind=='revolute' else 30),('d_gain',.15 if kind=='revolute' else .3),('cmd_max',8 if kind=='revolute' else 3),('cmd_min',-8 if kind=='revolute' else -3)]:add(controller,key,val)
 state=add(sdf_model,'plugin',filename='ignition-gazebo-joint-state-publisher-system',name='ignition::gazebo::systems::JointStatePublisher');add(state,'topic','/ghost/arm/joint_states')
 for name in JOINTS:add(state,'joint_name',name)
 # Gazebo cameras look along local +X; positive pitch points +X downwards.
 camera_link=sdf_model.find("link[@name='grasp_camera_link']")
 sensor=add(camera_link,'sensor',name='grasp_rgbd',type='rgbd_camera')
 add(sensor,'pose','0 0 -0.02 0 1.57079632679 0');add(sensor,'topic','/grasp_camera');add(sensor,'always_on','true');add(sensor,'update_rate',5)
 cam=add(sensor,'camera');add(cam,'horizontal_fov',1.2217304764);add(cam,'optical_frame_id','grasp_camera_optical_frame')
 im=add(cam,'image');add(im,'width',320);add(im,'height',240);add(im,'format','R8G8B8')
 clip=add(cam,'clip');add(clip,'near',.05);add(clip,'far',4)
 optical=add(robot,'link',name='grasp_camera_optical_frame')
 j=add(robot,'joint',name='grasp_optical_fixed',type='fixed');add(j,'parent',link='grasp_camera_link');add(j,'child',link='grasp_camera_optical_frame');add(j,'origin',xyz='0 0 -0.02',rpy='3.14159265359 0 -1.57079632679')
```

| 行号 | 逐句说明 |
| --- | --- |
| 1 | 模块说明：Gazebo 和 ROS 共用一份机械几何定义，使用米和弧度。 |
| 2 | 导入 Python 自带的 XML 库，缩写成 E；后面用它生成 SDF 和 URDF。 |
| 4 | 列出六个活动关节。列表顺序也是姿态数组的顺序：尾根、肩、肘、腕、左指、右指。 |
| 5 | 注释说明每个 PARTS 元组的十个字段顺序，避免把位置、尺寸、质量混淆。 |
| 6 | 开始结构参数列表；每一行定义一条父连杆到子连杆的连接。 |
| 7 | 固定底部安装座：相对球心向下 0.25 m；盒体长宽高为 0.24、0.18、0.035 m，质量 0.08 kg。fixed 的 axis 与 limits 是占位字段，不生成活动轴。 |
| 8 | 相机外壳固定在安装座前方 0.065 m、下方 0.04 m；盒体为 0.065×0.10×0.035 m，质量 0.04 kg。这里先定义外壳，后面才添加真正的传感器。 |
| 9 | 尾根支架固定在球心后方 0.27 m、下方 0.04 m；盒体中心另向后偏 0.025 m。这区分了连杆坐标原点与几何中心。 |
| 10 | 尾根旋转关节在支架后方 0.06 m、下方 0.04 m；绕 Z 轴旋转，限位 ±1.3 rad；其子连杆是 tail_yaw_link。 |
| 11 | 肩关节在尾根子连杆下方 0.045 m，绕 Y 轴旋转；限位 -1.6～0.7 rad。上臂长 0.22 m，几何中心在关节下方 0.11 m，因此上端位于关节处。 |
| 12 | 肘关节在上臂下端，即上臂原点下方 0.22 m；前臂长 0.20 m，中心下移 0.10 m，关节角限位 ±1.8 rad。 |
| 13 | 腕关节在前臂下端，绕 Y 轴旋转，限位 ±2.2 rad；子连杆是夹爪掌部，几何中心下移 0.035 m。 |
| 14 | 左指从掌部 Y=0.018、Z=-0.06 m 处出发，沿 +Y 方向滑动 0～0.03 m。指长 0.085 m、厚度 0.016 m，质量 0.02 kg。 |
| 15 | 右指初始位置 Y=-0.018 m，运动轴改为 -Y。因此两指同时输入正位移时向两侧张开，而不是一起向左移动。 |
| 16 | 在掌部下方 0.115 m 建立抓取中心。size=None 表示没有外观和碰撞盒；SDF 中仍给出 0.001 kg 的小质量与正惯量，它不是完全无物理影响的虚拟实体。 |
| 17 | 结束结构列表。上面的排列保证父连杆先于子连杆定义。 |
| 19 | 定义 XML 辅助函数：p 是父元素，t 是标签名，v 是标签内文本，**kw 收集 name、type 等 XML 属性。 |
| 20 | 在父元素下建立子元素，将 kw 字典写入 XML 属性。 |
| 21 | 如果提供了文本值，就转成字符串。必须用 is not None，才能保留数值 0。 |
| 22 | 返回新元素，方便继续在它下面创建子标签。 |
| 24 | 将元组中的每个数转为字符串，再用空格连接，例如 (0,0,-0.25) 变成 SDF 可读的坐标文本。 |
| 26 | attach 接收 SDF 的机器人模型和 URDF 的 robot 根元素，把同一结构同时添加到两者中。 |
| 27 | 注释记录兼容处理：为避开当前 Fortress 场景序列化的 relative_to 相关问题，生成零位下的模型坐标。 |
| 28 | 只有所有零位关节坐标轴初始方向一致时，才能直接累加平移。以后给关节原点加入旋转，必须改成齐次变换连乘。 |
| 29 | 以 base_link 为模型原点，字典保存各连杆在零位时相对模型的位置。不是运行时的正运动学求解器。 |
| 30 | 逐条读取 PARTS，同时将十个字段解包到有含义的变量中。 |
| 31 | 在 SDF 模型中创建当前子连杆。 |
| 32 | zip 配对父位置和局部位移，逐轴相加，保存子连杆的零位模型坐标。 |
| 33 | SDF pose 的六个数是 x y z roll pitch yaw；这里后三个数为零，未设置 relative_to，位置相对于模型坐标。 |
| 34 | 创建 inertial；把惯性坐标原点放在盒体中心 centre；写入质量 mass。 |
| 35 | 创建惯量张量；a、b、c 是盒体三个尺寸。没有盒体时用 1 cm 小立方体尺寸计算正惯量。 |
| 36 | 按均匀长方体公式 Ixx=m(b²+c²)/12、Iyy=m(a²+c²)/12、Izz=m(a²+b²)/12 计算惯量；质心主轴与盒体轴一致，所以交叉项设为零。 |
| 37 | 在 URDF 中创建同名连杆。当前 URDF 用于显示和 TF，物理惯量在 SDF 中定义。 |
| 38 | 只有有 size 的零件才生成外观与碰撞盒，抓取中心跳过这部分。 |
| 39 | 相机和夹指使用深色，其他结构使用青色；四个数表示红、绿、蓝和不透明度。 |
| 40 | 同一尺寸分别生成 visual 和 collision，避免显示几何与碰撞几何不一致。 |
| 41 | 创建 SDF 外观或碰撞元素并取唯一名称；局部 pose 将几何中心放在 centre。 |
| 42 | 嵌套创建 geometry → box → size，将三个尺寸写入盒体。 |
| 43 | 生成 URDF 对应元素；URDF 用 origin 属性表示局部偏移，用 box 的 size 属性表示尺寸，写法与 SDF 不同。 |
| 44 | 材质只对 visual 有意义，碰撞几何不需要颜色。 |
| 45 | 给 SDF 设置环境光颜色和漫反射颜色。 |
| 46 | 给 URDF 设置同样的 RGBA 颜色，使 RViz 与 Gazebo 外观对应。 |
| 47 | 下面添加便于观察的关节轴承外形。 |
| 48 | 仅在旋转关节处添加轴承圆柱，不给固定连接和滑动夹指添加。 |
| 49 | 新建轴承 visual；圆柱默认沿 Z 轴。Y 轴关节处绕 X 旋转 π/2，使圆柱轴线与关节轴线平行；正负方向不影响对称圆柱外观。 |
| 50 | 建立半径 3.2 cm、长度 7.5 cm 的圆柱轴承。 |
| 51 | 把轴承设为金黄色，便于区分关节和连杆。它只是视觉装饰，没有在这里单独增加碰撞体。 |
| 52 | 在 URDF 中生成相同圆柱：先建 visual，再设局部姿态、圆柱尺寸，最后设置金色材质。 |
| 53 | 生成 SDF joint，指定名称、类型、父连杆和子连杆。未单独设置 joint pose 时，关节原点采用子连杆原点。 |
| 54 | 生成 URDF joint；origin 描述零位时子关节坐标相对父连杆的位置，而不是盒体中心位置。 |
| 55 | 固定关节不需要活动轴、限位或控制器，因此下面只处理活动关节。 |
| 56 | 创建 SDF axis，写运动方向，并创建 limit 容器。 |
| 57 | 写入上下限、最大作用力/力矩以及速度限制：旋转关节为 8 N·m、1 rad/s，夹指为 3 N、0.06 m/s；限位本身不能替代完整轨迹控制。 |
| 58 | 给轴添加 0.08 的阻尼系数，用于抑制运动振荡；它不是 PID 的 D 增益。 |
| 59 | 写 URDF 轴和限位。当前 URDF 统一填写 effort=8，但实际物理仿真使用 SDF 中夹指的 3 N；以后用于 ros2_control 时应统一这些参数。 |
| 60 | 为每个活动关节加载 Gazebo JointPositionController，它接收目标关节位置并驱动物理关节。 |
| 61 | 指定关节与标准命令话题；/0/ 表示该关节第 0 根运动轴。旋转关节使用 P=5、D=0.15，夹指使用 P=30、D=0.3；cmd_max/min 限制输出范围。这里是位置控制器增益，不能与 arm_pose.py 的插值函数混淆。 |
| 62 | 创建实际关节状态发布插件，并指定 Gazebo 状态话题 /ghost/arm/joint_states。 |
| 63 | 逐个登记六个活动关节，使状态消息包含这些关节的反馈。 |
| 64 | 提醒：Gazebo 相机看向局部 +X，而 ROS 光学坐标的前向通常是 +Z；需要显式换轴。 |
| 65 | 按 name 找到已生成的相机外壳 link。 |
| 66 | 在外壳下挂载真正的 rgbd_camera 传感器，同时产生彩色和深度数据。 |
| 67 | 传感器在外壳下方 2 cm，绕 Y 转 π/2 朝下；话题前缀 /grasp_camera；保持启用，请求更新频率 5 Hz，实际频率受仿真性能影响。 |
| 68 | 设置约 70° 的水平视场，并标记图像对应的光学坐标系名称。后续三维反投影应读取实际 camera_info，不能只凭这里的角度假定内参。 |
| 69 | 图像为 320×240 像素，彩色格式为每通道 8 位的 RGB。深度图的编码应以收到的 ROS 消息为准，本机测试为浮点深度。 |
| 70 | 设置近裁剪面 0.05 m、远裁剪面 4 m；超出范围的数据不能直接当有效测距使用。 |
| 71 | 在 URDF 中创建没有几何的光学坐标 link。 |
| 72 | 以固定关节连接相机外壳与光学坐标；平移对应传感器安装位置，RPY=(π,0,-π/2) 完成朝下光学坐标的换轴。它与前面传感器 pose 的数值不同，因为两种相机轴约定不同。 |

### 结构参数怎样改

以肩关节行为例：关节原点、盒体中心和下一关节位置是三件事。把上臂从 0.22 m 改成 0.26 m 时，需要同步修改上臂 size 的 Z 分量、上臂 centre 的 Z 值为 -0.13，以及肘关节 origin 的 Z 值为 -0.26。否则会出现连杆与关节断开的外观。

两指间净开口约为 `2×0.018 - 0.016 + q_left + q_right`，也就是 `0.02 + q_left + q_right` 米。因此两指为零时仍有 2 cm 开口；这不是能够夹住任意薄物体的完全闭合夹爪。

参数列表不是 DH 参数表。当前是按父子连接关系生成模型，后续 DH 需要重新定义各轴的坐标系并验证与该模型一致。

## 3. 终端姿态控制：arm_pose.py

源文件：`src/ghost_sim/control/arm_pose.py`

```python
"""Slow, joint-space structural demonstrations; not an IK or grasp controller."""
import argparse
import time
import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from ghost_sim.simulation.tail_structure import JOINTS

PRESETS = {
 'hang': [0,0,0,0,.025,.025],
 'ready': [0,-.8,-.6,1.4,.025,.025],
 'close': [0,-.8,-.6,1.4,0,0],
}

def main():
 parser=argparse.ArgumentParser(description=__doc__)
 parser.add_argument('pose',choices=PRESETS)
 args=parser.parse_args()
 rclpy.init();node=rclpy.create_node('ghost_arm_pose');state={}
 sub=node.create_subscription(JointState,'/joint_states',lambda m:state.update(zip(m.name,m.position)),10)
 pubs=[node.create_publisher(Float64,'/ghost/arm/'+j+'/cmd_pos',10) for j in JOINTS]
 try:
  deadline=time.monotonic()+15
  while not all(j in state for j in JOINTS) or not all(p.get_subscription_count() for p in pubs):
   rclpy.spin_once(node,timeout_sec=.1)
   if time.monotonic()>deadline:raise RuntimeError('关节状态或控制桥未就绪，请先启动仿真。')
  start=[state[j] for j in JOINTS];target=PRESETS[args.pose];began=time.monotonic();duration=4
  while time.monotonic()-began<duration+8:
   u=min(1,(time.monotonic()-began)/duration);blend=u*u*(3-2*u)
   for p,a,b in zip(pubs,start,target):p.publish(Float64(data=float(a+(b-a)*blend)))
   rclpy.spin_once(node,timeout_sec=.02);time.sleep(.02)
   errors=[abs(state[j]-t) for j,t in zip(JOINTS,target)]
   if u==1 and all(e<(.002 if i>=4 else .035) for i,e in enumerate(errors)):
    print('姿态完成：',args.pose,{j:round(state[j],4) for j in JOINTS});return
  raise RuntimeError('关节未在时限内到达目标：'+str(state))
 finally:node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
```

| 行号 | 逐句说明 |
| --- | --- |
| 1 | 说明程序用于慢速关节空间演示，不计算逆运动学，也不判断抓取是否成功。 |
| 2 | 导入命令行参数解析库。 |
| 3 | 导入计时与休眠工具。 |
| 4 | 导入 ROS 2 Python 客户端。 |
| 5 | JointState 消息包含关节名以及位置、速度等反馈数组。 |
| 6 | Float64 消息只有一个双精度 data 字段，用于单关节目标位置。 |
| 7 | 复用结构定义中的关节顺序，避免控制顺序与模型不一致。 |
| 9 | 定义姿态字典：键是命令名称，值是六个关节目标。 |
| 10 | hang：四个旋转关节设零，两指各外移 2.5 cm，尾臂下垂、夹爪张开。 |
| 11 | ready：尾根 0，肩 -0.8，肘 -0.6，腕 1.4 rad；俯仰角总和为零，使夹爪在这一结构中朝下。两指保持张开。 |
| 12 | close：保持同一前伸姿态，只将两指设为零；它会把任意当前姿态移动到 ready 后闭合，不是只动手指的独立命令。 |
| 13 | 结束姿态字典。 |
| 15 | 主函数是程序入口。 |
| 16 | 建立解析器，使用模块说明作为帮助文本。 |
| 17 | 添加必填 pose 参数，只允许姿态字典中已有的名称。 |
| 18 | 读取用户在命令行输入的参数。 |
| 19 | 初始化 ROS；创建 ghost_arm_pose 节点；创建保存反馈位置的字典。 |
| 20 | 订阅 /joint_states，队列深度为 10；回调使用 zip 将关节名与位置配对，再更新 state。 |
| 21 | 按照六个关节名分别建立 Float64 发布器，话题形式为 /ghost/arm/关节名/cmd_pos。 |
| 22 | 进入 try/finally，保证退出时清理 ROS 资源。 |
| 23 | 用单调时钟设置 15 秒连接等待上限；系统时间校正不会让单调时钟倒退。 |
| 24 | 只要还有关节状态缺失，或者某个发布器没有订阅者，就继续等待。订阅者一般是 ROS/Gazebo 桥，不等于已经保证控制器正常。 |
| 25 | 处理一次 ROS 回调，最多等待 0.1 秒，以便接收状态和发现订阅者。 |
| 26 | 超时则报告仿真或桥接未就绪，不直接在未知初始位置下发送动作。 |
| 27 | 保存实际起始角度/位移；查找目标；记录开始时间；将指令插值时长设为 4 秒。 |
| 28 | 最多运行 12 秒，即 4 秒插值加 8 秒到位等待。这里使用墙上实际时间，慢速仿真可能需要更久才能跟上。 |
| 29 | u 是 0～1 的时间进度；blend=3u²-2u³ 是三次平滑插值，起止斜率均为零。它平滑目标指令，但不保证实际关节严格按时运动。 |
| 30 | 对每个关节计算 a+(b-a)×blend，并发布。前三项变量 p/a/b 分别是发布器、初始值和目标值。 |
| 31 | 处理反馈回调，再休眠 0.02 秒以避免忙循环；不能把它理解为严格恒定 50 Hz。 |
| 32 | 对所有关节计算实际位置与最终目标的绝对误差。 |
| 33 | 只有插值完成且全部误差满足条件才算到位；四个转轴阈值为 0.035 rad，两指为 0.002 m。 |
| 34 | 打印实际反馈值并返回；round(...,4) 只影响打印精度。 |
| 35 | 超出等待时间则抛出异常，并打印最后记录的状态。 |
| 36 | 无论正常结束或异常都销毁节点并关闭 ROS 客户端。Gazebo 控制器会保留最后目标，退出客户端不等于撤销目标。 |
| 38 | 仅在作为程序执行时调用 main；被其他模块导入时不会自动运动。 |

### 当前控制的边界

这个程序没有轨迹避碰、反馈新鲜度看门狗或目标物接触检测；收到一次状态不表示之后一直有新反馈。六关节插值也不是笛卡尔直线运动。它适合当前的结构演示，后续抓取控制需要加入这些检查。

## 4. 启动各个节点：sim.launch.py

源文件：`launch/sim.launch.py`

```python
"""Ghost simulation, bridges, model display, camera and flight control."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ghost_sim.paths import WORLD, MODEL, CONFIG
from ghost_sim.simulation.tail_structure import JOINTS


def static_tf(name,parent,child,xyz,rpy=(0,0,0)):
    x,y,z = xyz
    roll,pitch,yaw = rpy
    return Node(package='tf2_ros',executable='static_transform_publisher',name=name,
                arguments=['--x',str(x),'--y',str(y),'--z',str(z),'--roll',str(roll),
                           '--pitch',str(pitch),'--yaw',str(yaw),'--frame-id',parent,'--child-frame-id',child])


def generate_launch_description():
    def gazebo(context):
        command = ['ign','gazebo','-r','-v','3','-s',str(WORLD)]
        processes = [ExecuteProcess(cmd=command,output='screen')]
        if LaunchConfiguration('gui').perform(context) == 'true':
            processes.append(TimerAction(period=3.0,actions=[ExecuteProcess(
                cmd=['ign','gazebo','-g','--gui-config',str(CONFIG/'arm_gazebo.config')],
                additional_env={'LIBGL_ALWAYS_SOFTWARE':'1','QT_OPENGL':'software','QSG_RENDER_LOOP':'basic'},
                output='screen')]))
        return processes

    actions = [DeclareLaunchArgument('gui',default_value='false'),OpaqueFunction(function=gazebo),
               Node(package='robot_state_publisher',executable='robot_state_publisher',
                    parameters=[{'robot_description':(MODEL).read_text(),'use_sim_time':True}])]
    for module in ['visualization.ground_truth_tf','visualization.reference_scene','visualization.cloud_frame','control.flight_control','mapping.voxel_map_node','navigation.navigation3d']:
        actions.append(ExecuteProcess(cmd=['python3','-m','ghost_sim.'+module,'--ros-args','-p','use_sim_time:=true'],output='screen'))
    actions.append(Node(package='ros_gz_bridge',executable='parameter_bridge',name='ghost_bridge',
                        arguments=[
                            '/model/ghost/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
                            '/model/ghost/pose@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
                            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                            '/ghost/arm/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
                            '/grasp_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
                            '/grasp_camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
                            '/grasp_camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
                            '/camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
                            '/camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
                            '/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
                            '/camera/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked'],
                        remappings=[('/ghost/arm/joint_states','/joint_states'),('/model/ghost/pose','/simulation/ground_truth/poses'),
                                    ('/camera/points','/camera/points_raw')],output='screen'))
    actions.append(Node(package='ros_gz_bridge',executable='parameter_bridge',name='arm_bridge',
                        parameters=[{'config_file':str(CONFIG/'arm_bridge.yaml')}],output='screen'))
    actions.extend([static_tf('camera_mount','base_link','camera_link',(.29,0,0)),
                    static_tf('camera_optical','camera_link','camera_optical_frame',(0,0,0),
                              (-1.57079632679,0,-1.57079632679))])
    return LaunchDescription(actions)
```

| 行号 | 逐句说明 |
| --- | --- |
| 1 | 说明启动范围包括仿真、桥接、显示、相机和飞行控制。 |
| 2 | 导入 Path；当前文件没有实际使用，属于保留的冗余导入。 |
| 3 | 导入 LaunchDescription，它是 ROS 启动动作的集合。 |
| 4 | 分别导入声明参数、启动进程、运行时执行函数和延时动作。 |
| 5 | 读取启动时传入的配置，如 gui:=true。 |
| 6 | Node 用于启动 ROS 节点，自动处理 ROS 参数和重映射。 |
| 7 | 从统一路径模块取得世界模型、URDF 和配置目录。 |
| 8 | JOINTS 在桥接改为 YAML 后不再用于本文件，当前为冗余导入。 |
| 11 | 定义构造静态 TF 发布节点的辅助函数；rpy 默认不旋转。 |
| 12 | 将平移元组拆成 x/y/z。 |
| 13 | 将姿态元组拆成滚转、俯仰和偏航。 |
| 14 | 创建 tf2_ros 的静态变换发布进程。 |
| 15 | 把平移与 roll 作为命令行参数传入；数值转字符串。 |
| 16 | 补齐 pitch/yaw 和父子坐标系名称，结束这个节点定义。 |
| 19 | ROS launch 会调用这个函数获得需要启动的动作。 |
| 20 | 定义在启动上下文已就绪时调用的 Gazebo 构造函数。 |
| 21 | -r 自动运行，-v 3 设置日志级别，-s 只启动服务端，最后的 WORLD 是 SDF 文件。 |
| 22 | 先创建服务端进程动作，日志打印到启动输出。 |
| 23 | 只有用户显式传入 gui:=true 才启动图形客户端。 |
| 24 | 延时 3 秒启动 GUI，避免本机服务端和图形客户端同时初始化导致的问题；这不是严格的就绪检测。 |
| 25 | -g 表示仅运行 GUI；加载项目专用配置，其中包括关节滑块面板和观察视角。 |
| 26 | 使用本机验证过的软件 OpenGL 与 Qt basic 渲染循环。这是虚拟机兼容配置，不会让 CPU 渲染获得 GPU 加速。 |
| 27 | 完成 GUI 进程与 TimerAction 的括号，日志也输出到屏幕。 |
| 28 | 返回服务端以及可选图形客户端动作。 |
| 30 | 声明 gui 参数，默认关闭；OpaqueFunction 让 gazebo 函数在能读取启动参数时执行。 |
| 31 | 启动 robot_state_publisher。 |
| 32 | 读取 URDF 文本作为 robot_description，启用仿真时间；节点结合 /joint_states 计算并发布连杆 TF。 |
| 33 | 列出继承自幽灵球项目的模块：真值 TF、场景显示、点云坐标修正、飞行控制、体素地图和导航。 |
| 34 | 用 python3 -m 启动各模块，并传入 use_sim_time。保留启动这些节点不代表旧导航已考虑新尾臂外形。 |
| 35 | 建立通用 ROS/Gazebo 参数桥。 |
| 36 | 开始桥接声明列表：话题、ROS 类型和 Gazebo 类型按约定字符串拼接。 |
| 37 | 分隔符 ] 表示 ROS → Gazebo；将球体速度指令送入仿真。 |
| 38 | 分隔符 [ 表示 Gazebo → ROS；把模型物理位姿转换成 TFMessage。 |
| 39 | 把 Gazebo 时钟转成 ROS /clock，供 use_sim_time 节点使用。 |
| 40 | 将 Gazebo Model 中的关节状态转换为 ROS JointState。 |
| 41 | 桥接底部相机彩色图。 |
| 42 | 桥接底部相机深度图。 |
| 43 | 桥接底部相机内参。 |
| 44 | 保留前视相机彩色图。 |
| 45 | 保留前视相机深度图。 |
| 46 | 保留前视相机内参。 |
| 47 | 将前视相机的点云转换为 PointCloud2，并结束桥接参数列表。 |
| 48 | 把 ROS 侧关节反馈改名为标准 /joint_states；模型位姿改为现有真值订阅者需要的名称。 |
| 49 | 将原始点云重命名为 /camera/points_raw，供坐标修正节点处理。 |
| 50 | 另外启动 arm_bridge，专门传递六个关节的位置命令。 |
| 51 | 读取 YAML 配置，允许 ROS 与 Gazebo 话题名称不同，避开 /0/ 这种 ROS 不接受的名称段。 |
| 52 | 发布原前视相机安装坐标：在球心前方 0.29 m。 |
| 53 | 为前视相机建立光学坐标变换，平移为零。 |
| 54 | 通过固定 RPY 旋转换成前视相机光学轴约定；底部相机的固定变换已在 URDF 中定义，不能再重复发布。 |
| 55 | 返回全部启动动作。 |

## 5. 逐行解释关节桥：arm_bridge.yaml

源文件：`config/arm_bridge.yaml`

```yaml
- ros_topic_name: /ghost/arm/tail_yaw/cmd_pos
  gz_topic_name: /model/ghost/joint/tail_yaw/0/cmd_pos
  ros_type_name: std_msgs/msg/Float64
  gz_type_name: ignition.msgs.Double
  direction: ROS_TO_GZ
- ros_topic_name: /ghost/arm/tail_shoulder/cmd_pos
  gz_topic_name: /model/ghost/joint/tail_shoulder/0/cmd_pos
  ros_type_name: std_msgs/msg/Float64
  gz_type_name: ignition.msgs.Double
  direction: ROS_TO_GZ
- ros_topic_name: /ghost/arm/tail_elbow/cmd_pos
  gz_topic_name: /model/ghost/joint/tail_elbow/0/cmd_pos
  ros_type_name: std_msgs/msg/Float64
  gz_type_name: ignition.msgs.Double
  direction: ROS_TO_GZ
- ros_topic_name: /ghost/arm/tail_wrist/cmd_pos
  gz_topic_name: /model/ghost/joint/tail_wrist/0/cmd_pos
  ros_type_name: std_msgs/msg/Float64
  gz_type_name: ignition.msgs.Double
  direction: ROS_TO_GZ
- ros_topic_name: /ghost/arm/finger_left_slide/cmd_pos
  gz_topic_name: /model/ghost/joint/finger_left_slide/0/cmd_pos
  ros_type_name: std_msgs/msg/Float64
  gz_type_name: ignition.msgs.Double
  direction: ROS_TO_GZ
- ros_topic_name: /ghost/arm/finger_right_slide/cmd_pos
  gz_topic_name: /model/ghost/joint/finger_right_slide/0/cmd_pos
  ros_type_name: std_msgs/msg/Float64
  gz_type_name: ignition.msgs.Double
  direction: ROS_TO_GZ
```

| 行号 | 逐句说明 |
| --- | --- |
| 1 | 这一组的 ROS 订阅话题为 `/ghost/arm/tail_yaw/cmd_pos`，终端姿态程序向它发布。 |
| 2 | 对应 Gazebo 发布话题为 `/model/ghost/joint/tail_yaw/0/cmd_pos`，与 GUI 和位置控制器使用的话题一致。 |
| 3 | ROS 消息使用 Float64，data 字段携带角度或位移。 |
| 4 | Gazebo 使用 Double 消息，桥负责转换消息格式。 |
| 5 | 方向仅为 ROS_TO_GZ；实际关节状态通过另一条状态桥返回，不把目标命令当反馈。 |
| 6 | 这一组的 ROS 订阅话题为 `/ghost/arm/tail_shoulder/cmd_pos`，终端姿态程序向它发布。 |
| 7 | 对应 Gazebo 发布话题为 `/model/ghost/joint/tail_shoulder/0/cmd_pos`，与 GUI 和位置控制器使用的话题一致。 |
| 8 | ROS 消息使用 Float64，data 字段携带角度或位移。 |
| 9 | Gazebo 使用 Double 消息，桥负责转换消息格式。 |
| 10 | 方向仅为 ROS_TO_GZ；实际关节状态通过另一条状态桥返回，不把目标命令当反馈。 |
| 11 | 这一组的 ROS 订阅话题为 `/ghost/arm/tail_elbow/cmd_pos`，终端姿态程序向它发布。 |
| 12 | 对应 Gazebo 发布话题为 `/model/ghost/joint/tail_elbow/0/cmd_pos`，与 GUI 和位置控制器使用的话题一致。 |
| 13 | ROS 消息使用 Float64，data 字段携带角度或位移。 |
| 14 | Gazebo 使用 Double 消息，桥负责转换消息格式。 |
| 15 | 方向仅为 ROS_TO_GZ；实际关节状态通过另一条状态桥返回，不把目标命令当反馈。 |
| 16 | 这一组的 ROS 订阅话题为 `/ghost/arm/tail_wrist/cmd_pos`，终端姿态程序向它发布。 |
| 17 | 对应 Gazebo 发布话题为 `/model/ghost/joint/tail_wrist/0/cmd_pos`，与 GUI 和位置控制器使用的话题一致。 |
| 18 | ROS 消息使用 Float64，data 字段携带角度或位移。 |
| 19 | Gazebo 使用 Double 消息，桥负责转换消息格式。 |
| 20 | 方向仅为 ROS_TO_GZ；实际关节状态通过另一条状态桥返回，不把目标命令当反馈。 |
| 21 | 这一组的 ROS 订阅话题为 `/ghost/arm/finger_left_slide/cmd_pos`，终端姿态程序向它发布。 |
| 22 | 对应 Gazebo 发布话题为 `/model/ghost/joint/finger_left_slide/0/cmd_pos`，与 GUI 和位置控制器使用的话题一致。 |
| 23 | ROS 消息使用 Float64，data 字段携带角度或位移。 |
| 24 | Gazebo 使用 Double 消息，桥负责转换消息格式。 |
| 25 | 方向仅为 ROS_TO_GZ；实际关节状态通过另一条状态桥返回，不把目标命令当反馈。 |
| 26 | 这一组的 ROS 订阅话题为 `/ghost/arm/finger_right_slide/cmd_pos`，终端姿态程序向它发布。 |
| 27 | 对应 Gazebo 发布话题为 `/model/ghost/joint/finger_right_slide/0/cmd_pos`，与 GUI 和位置控制器使用的话题一致。 |
| 28 | ROS 消息使用 Float64，data 字段携带角度或位移。 |
| 29 | Gazebo 使用 Double 消息，桥负责转换消息格式。 |
| 30 | 方向仅为 ROS_TO_GZ；实际关节状态通过另一条状态桥返回，不把目标命令当反馈。 |

## 6. 原模型生成器新增了什么

`src/ghost_sim/simulation/create_scene.py` 原来生成房间和幽灵球，现在新增：

```python
from ghost_sim.simulation.tail_structure import attach
```

导入机械结构生成函数。导入本身不生成模型，只让当前文件能调用它。

```python
attach(ghost, urdf)
E.indent(sdf)
E.ElementTree(sdf).write(WORLD, encoding='unicode', xml_declaration=True)
E.indent(urdf)
E.ElementTree(urdf).write(MODEL, encoding='unicode')
```

- `attach(ghost, urdf)`：在已建立的球体模型与显示模型中同时添加尾臂、夹爪、底部相机；必须在写文件之前调用。
- `E.indent(sdf)`：格式化缩进，便于打开 SDF 阅读，不改变机械结构。
- `E.ElementTree(sdf)`：把 XML 根元素包装成整棵树。
- `.write(WORLD, ...)`：写入 `assets/worlds/ghost_room.sdf`，包含物理模型、传感器与插件；运行中的 Gazebo 不会自动重新读取这个文件。
- `E.indent(urdf)`：格式化显示模型。
- `.write(MODEL, ...)`：写入 `assets/models/ghost_display.urdf`，供 robot_state_publisher 使用。

球体材质 alpha 调整为 1，表示不透明。相机渲染与 GUI 渲染的兼容性处理不改变球体碰撞半径。

生成文件中重点看这些元素：

```xml
<joint name="tail_shoulder" type="revolute">
  <parent>tail_yaw_link</parent>
  <child>tail_upper</child>
  <axis>
    <xyz>0 1 0</xyz>
    <limit>
      <lower>-1.6</lower>
      <upper>0.7</upper>
      <effort>8</effort>
      <velocity>1</velocity>
    </limit>
  </axis>
</joint>
```

`joint` 声明旋转连接；`parent/child` 指定两端连杆；`axis/xyz` 指定转动轴；`lower/upper` 指定关节位置范围；`effort` 为力矩约束；`velocity` 为角速度约束。这里摘录主要字段，完整生成文件还包含阻尼等信息。不要只改生成文件，否则下次 build-model 会覆盖它。

## 7. Gazebo 内的关节控制面板

`config/arm_gazebo.config` 在原 GUI 配置基础上加入：

```xml
<plugin filename="JointPositionController" name="Tail and gripper control">
  <ignition-gui>
    <title>Tail and gripper control</title>
    <property type="string" key="state">docked</property>
  </ignition-gui>
  <model_name>ghost</model_name>
</plugin>
```

逐行说明：

1. 加载 GUI 的 JointPositionController 面板。它与 SDF 中同名的系统控制器属于不同插件：面板发命令，系统控制器执行命令。
2. `ignition-gui` 包住界面属性。
3. `title` 是面板上显示的标题。
4. `state=docked` 让面板停靠在 Gazebo 窗口中。
5. 结束界面属性。
6. `model_name=ghost` 启动后自动锁定幽灵模型，列出它的活动关节。
7. 结束插件定义。

GUI 中每个滑块从模型获取限位。旋转关节单位 rad，夹指单位 m；夹指输入框可能只显示两位小数，0.025 m 会被显示为约 0.03 m，这不表示实际命令一定为 0.03 m。

配置中的 `<engine>ogre2</engine>` 选择渲染引擎；`camera_pose` 是观察场景的 GUI 相机姿态，和机器人底部的 RGB-D 相机没有关系。ComponentInspector 与 EntityTree 没有默认加载，以便给六个滑块留出空间，需要时可从 GUI 插件菜单添加。

## 8. 为什么改项目目录后还能运行

```python
ROOT = Path(__file__).resolve().parents[2]
```

`__file__` 是 `src/ghost_sim/paths.py` 自身位置；`resolve()` 得到绝对路径；`parents[0]` 是 `src/ghost_sim`，`parents[1]` 是 `src`，`parents[2]` 就是 `/home/ubuntu/ghost_arm`。资源路径随后使用 ROOT 拼接，不依赖启动终端所在目录。

```bash
GHOST_ENTRY_ROOT="$(cd "$(dirname "$0")" && pwd)"
```

`$0` 是启动脚本路径；`dirname` 取所在目录；`cd` 进入那个目录，`pwd` 得到绝对路径；命令替换 `$(...)` 把结果赋给变量。因此脚本放到新目录后会自动找到新目录。

```bash
source "$GHOST_ENTRY_ROOT/scripts/env.sh"
exec python3 -m ghost_sim.control.arm_pose "$@"
```

第一句把 ROS 环境与项目 `src` 路径加载到当前 shell。第二句执行 Python 模块；`-m` 使用包导入机制；`"$@"` 原样传递剩余参数，如 ready；`exec` 让 Python 替换当前 shell 进程。

`env.sh` 保留 ROS_DOMAIN_ID=43 和 IGN_PARTITION=ghost_sim，启动锁也与原项目共用，避免同时启动两个相同话题的仿真。内部变量名 GHOST_SIM_ROOT 不影响新项目名，它的值会指向脚本实际所在项目。

## 9. 从零手动完成的建议顺序

1. 先写 PARTS 中的底座与一根连杆，生成 SDF，检查尺寸和坐标。
2. 添加一个 revolute 关节和位置控制器，单独测试角度命令。
3. 按父子顺序增加肩、肘、腕，验证零位、限位和旋转正方向。
4. 添加两个相反轴向的滑动夹指，检查净开口。
5. 添加实际关节状态桥，再让 URDF 和 TF 跟随物理模型。
6. 添加朝下 RGB-D 相机，确认图像、深度和 frame_id。
7. 添加 GUI 面板与 YAML 命令桥。
8. 最后写姿态插值程序，并通过反馈而不是目测判断到位。

## 10. 运行与验证

```bash
cd /home/ubuntu/ghost_arm
./ghost.sh build-model       # 仅修改几何参数后需要重新生成
./ghost.sh start gui:=true   # 启动新项目
```

另开终端：

```bash
cd /home/ubuntu/ghost_arm
./ghost.sh arm ready
./ghost.sh arm close
./ghost.sh arm hang
```

调试反馈：

```bash
source /home/ubuntu/ghost_arm/scripts/env.sh
ros2 topic echo /joint_states --once
ros2 topic info /grasp_camera/depth_image
```

本次已验证迁移后的 17 项原有单元测试和 ready 实际关节动作。之前还验证过 Gazebo 滑块能够转动尾根与闭合两指。自动抓取、受重力物体、DH 推导、逆运动学和尾臂避碰仍属于下一阶段；现有导航只使用原球形占用范围。

参考现有安装版本的官方实现：[Gazebo 关节位置控制器](https://gazebosim.org/api/gazebo/6/classignition_1_1gazebo_1_1systems_1_1JointPositionController.html)。本文件对行为的解释以项目当前代码为准。

## DH 与正运动学

已实现标准 DH 四轴正运动学；使用 `./ghost.sh fk --joints 0 -0.8 -0.6 1.4` 离线计算，或 `./ghost.sh fk --live` 与 Gazebo 同时刻实际位姿比较。详见 [DH 参数与正运动学逐句讲解](DH参数与正运动学逐句讲解.md)。

## 解析逆运动学（新增）

已实现位置＋俯仰角解析逆解，包括双肘分支、限位、奇异提示与参考姿态选解。运行 `./ghost.sh ik 0.024908286 0 -0.427268905 --pitch 0 --live` 读取当前角度选解；省略 `--live` 可离线计算。命令只计算，不运动，尚未做尾臂碰撞检测。详见 [逆运动学逐句讲解](逆运动学逐句讲解.md)。

## 坐标目标执行（新增）

`./ghost.sh ik 0.02 0.04 -0.43 --pitch 0 --execute` 会求逆解并实际运动。省略 `--execute` 保持只计算。历史章节中的“只读”说明针对默认模式。详见 [坐标逆解与运动执行](坐标逆解与运动执行.md)。

## 底部 RGB-D 测量

运行 `./ghost.sh camera` 查看底部彩色图与深度图，点击左侧 RGB 得到球体坐标系中的表面点；`./ghost.sh camera --pixel 100 80` 单次测量。不会自动移动机械臂。详见 [相机接入与代码讲解](底部深度相机接入与逐句讲解.md)。
