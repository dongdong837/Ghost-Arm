"""Shared mechanical geometry for Gazebo and ROS; metres, radians, +X forward."""
import xml.etree.ElementTree as E
import math

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
  if kind!='fixed':add(sl,'self_collide','true')
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
    if kind_geom=='collision' and 'finger' in child:
     friction=add(add(add(sg,'surface'),'friction'),'ode');add(friction,'mu',1.5);add(friction,'mu2',1.5)
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
   if kind=='prismatic':
    # A blocked finger has intentional position error; never integrate it into a persistent closing force.
    for key in ('i_gain','i_max','i_min'):add(controller,key,0)
   for key,val in [('joint_name',name),('topic','/model/ghost/joint/'+name+'/0/cmd_pos'),('p_gain',5 if kind=='revolute' else 30),('d_gain',.15 if kind=='revolute' else .3),('cmd_max',8 if kind=='revolute' else 3),('cmd_min',-8 if kind=='revolute' else -3)]:add(controller,key,val)
 state=add(sdf_model,'plugin',filename='libghost_joint_feedback.so',name='ghost_arm::JointFeedback');add(state,'topic','/ghost/arm/joint_states')
 for name in JOINTS:add(state,'joint_name',name)
 # Gazebo cameras look along local +X; positive pitch points +X downwards.
 camera_link=sdf_model.find("link[@name='grasp_camera_link']")
 sensor=add(camera_link,'sensor',name='grasp_rgbd',type='rgbd_camera')
 add(sensor,'pose','0 0 -0.02 0 1.57079632679 0');add(sensor,'topic','/grasp_camera');add(sensor,'always_on','true');add(sensor,'update_rate',5)
 cam=add(sensor,'camera');add(cam,'horizontal_fov',1.2217304764);add(cam,'optical_frame_id','grasp_camera_optical_frame')
 # Explicit intrinsics avoid Fortress defaults (fx=277) disagreeing with the 70-degree image.
 focal=160/math.tan(1.2217304764/2)
 intrinsics=add(add(cam,'lens'),'intrinsics')
 for key,value in [('fx',focal),('fy',focal),('cx',159.5),('cy',119.5),('s',0)]:add(intrinsics,key,value)
 im=add(cam,'image');add(im,'width',320);add(im,'height',240);add(im,'format','R8G8B8')
 clip=add(cam,'clip');add(clip,'near',.05);add(clip,'far',4)
 optical=add(robot,'link',name='grasp_camera_optical_frame')
 j=add(robot,'joint',name='grasp_optical_fixed',type='fixed');add(j,'parent',link='grasp_camera_link');add(j,'child',link='grasp_camera_optical_frame');add(j,'origin',xyz='0 0 -0.02',rpy='3.14159265359 0 -1.57079632679')
