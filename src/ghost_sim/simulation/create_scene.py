"""Build a floating ghost and its RViz model; room geometry is kept locally."""
from pathlib import Path
import xml.etree.ElementTree as E
from ghost_sim.paths import WORLD, MODEL, TEMPLATE
from ghost_sim.simulation.tail_structure import attach
from ghost_sim.simulation.wood_block import attach as attach_wood

def sub(parent, tag, text=None, **attrs):
    node = E.SubElement(parent, tag, attrs)
    if text is not None:
        node.text = str(text)
    return node

def visual(link, name, radius, pose, color):
    node = sub(link, 'visual', name=name)
    sub(node, 'pose', pose)
    sub(sub(sub(node, 'geometry'), 'sphere'), 'radius', radius)
    material = sub(node, 'material')
    sub(material, 'ambient', color)
    sub(material, 'diffuse', color)
    return node

def plugin(parent, filename, name):
    return sub(parent, 'plugin', filename='ignition-gazebo-'+filename+'-system',
               name='ignition::gazebo::systems::'+name)

def build():
    sdf = E.parse(TEMPLATE).getroot()
    world = sdf.find('world')
    world.set('name', 'ghost_room')
    # Fortress/DART does not reliably honour per-link gravity disable here.
    # Keep the ghost weightless; wood_block receives its own weight force.
    gravity = world.find('gravity')
    if gravity is None:
        gravity = sub(world,'gravity')
    gravity.text = '0 0 0'
    for model in list(world.findall('model')):
        if model.findtext('static') != 'true':
            world.remove(model)
        elif model.get('name') in ('north','south','east','west'):
            values = model.findtext('pose').split()
            values[2] = '1.5'
            model.find('pose').text = ' '.join(values)
            for size in model.findall('.//geometry/box/size'):
                values = size.text.split(); values[2] = '3.0'
                size.text = ' '.join(values)
    roof = sub(world, 'model', name='ceiling')
    sub(roof,'static','true'); sub(roof,'pose','0 0 3.05 0 0 0')
    roof_link = sub(roof,'link',name='link')
    for kind in ('collision','visual'):
        part = sub(roof_link,kind,name=kind)
        sub(sub(sub(part,'geometry'),'box'),'size','8 6 0.1')
        if kind == 'visual':
            mat = sub(part,'material')
            sub(mat,'diffuse','0.7 0.8 0.95 0.08')
            sub(mat,'ambient','0.7 0.8 0.95 0.08')
            sub(part,'transparency',0.92)
    ghost = sub(world,'model',name='ghost')
    sub(ghost,'pose','0 0 1.2 0 0 0')
    link = sub(ghost,'link',name='base_link')
    sub(link,'gravity','false')
    inertial = sub(link,'inertial'); sub(inertial,'mass',1.0)
    tensor = sub(inertial,'inertia')
    for name,value in [('ixx',0.03136),('iyy',0.03136),('izz',0.03136),('ixy',0),('ixz',0),('iyz',0)]:
        sub(tensor,name,value)
    collision = sub(link,'collision',name='sphere_collision')
    sub(sub(sub(collision,'geometry'),'sphere'),'radius',0.28)
    visual(link,'ghost_body',0.25,'0 0 0 0 0 0','0.65 0.9 1.0 1')
    for name,y in [('left',0.075),('right',-0.075)]:
        visual(link,name+'_eye',0.035,f'0.225 {y} 0.05 0 0 0','1 1 1 1')
        visual(link,name+'_pupil',0.016,f'0.253 {y} 0.05 0 0 0','0.025 0.04 0.10 1')
    visual(link,'mouth',0.018,'0.245 0 -0.065 0 0 0','0.06 0.10 0.20 1')
    velocity = plugin(ghost,'velocity-control','VelocityControl')
    sub(velocity,'topic','/model/ghost/cmd_vel')
    sub(velocity,'initial_linear','0 0 0'); sub(velocity,'initial_angular','0 0 0')
    poses = plugin(ghost,'pose-publisher','PosePublisher')
    for k,v in [('publish_link_pose','true'),('publish_model_pose','true'),
                ('publish_nested_model_pose','true'),('use_pose_vector_msg','true'),('update_frequency',50)]:
        sub(poses,k,v)
    sensor = sub(link,'sensor',name='rgbd',type='rgbd_camera')
    sub(sensor,'pose','0.29 0 0 0 0 0'); sub(sensor,'topic','/camera')
    sub(sensor,'always_on','true'); sub(sensor,'update_rate',5)
    camera = sub(sensor,'camera'); sub(camera,'horizontal_fov',1.0471975512)
    sub(camera,'optical_frame_id','camera_optical_frame')
    image = sub(camera,'image')
    for k,v in [('width',320),('height',240),('format','R8G8B8')]: sub(image,k,v)
    clip = sub(camera,'clip'); sub(clip,'near',0.1); sub(clip,'far',8.0)
    # Generate the same visual spheres for ROS robot_state_publisher / RViz.
    urdf = E.Element('robot',name='ghost_display')
    body = E.SubElement(urdf,'link',name='base_link')
    for source in link.findall('visual'):
        item = E.SubElement(body,'visual',name=source.get('name'))
        pose = source.findtext('pose').split()
        E.SubElement(item,'origin',xyz=' '.join(pose[:3]),rpy=' '.join(pose[3:]))
        geometry = E.SubElement(item,'geometry')
        E.SubElement(geometry,'sphere',radius=source.findtext('geometry/sphere/radius'))
        mat = E.SubElement(item,'material',name=source.get('name'))
        E.SubElement(mat,'color',rgba=source.findtext('material/diffuse'))
    attach(ghost, urdf)
    attach_wood(world)
    E.indent(sdf); E.ElementTree(sdf).write(WORLD,encoding='unicode',xml_declaration=True)
    E.indent(urdf); E.ElementTree(urdf).write(MODEL,encoding='unicode')

if __name__ == '__main__':
    build()
