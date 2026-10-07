"""A dynamic wooden practice block with local gravity in the floating-ghost world."""
import xml.etree.ElementTree as E

SIZE = (.04, .04, .06)
MASS = .04
POSITION = (.10, 0., .03)


def add(parent, tag, value=None, **attributes):
    element = E.SubElement(parent, tag, attributes)
    if value is not None:
        element.text = str(value)
    return element


def attach(world):
    model = add(world, 'model', name='wood_block')
    add(model, 'static', 'false')
    add(model, 'pose', ' '.join(map(str, POSITION))+' 0 0 0')
    link = add(model, 'link', name='link')
    inertia = add(link, 'inertial')
    add(inertia, 'mass', MASS)
    tensor = add(inertia, 'inertia')
    x, y, z = SIZE
    for key, value in [('ixx', MASS*(y*y+z*z)/12), ('iyy', MASS*(x*x+z*z)/12),
                       ('izz', MASS*(x*x+y*y)/12), ('ixy', 0), ('ixz', 0), ('iyz', 0)]:
        add(tensor, key, value)
    for kind in ('visual', 'collision'):
        shape = add(link, kind, name='wood_'+kind)
        add(add(add(shape, 'geometry'), 'box'), 'size', ' '.join(map(str, SIZE)))
        if kind == 'visual':
            material = add(shape, 'material')
            add(material, 'ambient', '0.65 0.35 0.12 1')
            add(material, 'diffuse', '0.65 0.35 0.12 1')
        else:
            friction = add(add(add(shape, 'surface'), 'friction'), 'ode')
            add(friction, 'mu', 1.5)
            add(friction, 'mu2', 1.5)
    # Apply only this body's weight, leaving the existing floating robot unchanged.
    force = add(world, 'plugin', filename='ignition-gazebo-apply-link-wrench-system',
                name='ignition::gazebo::systems::ApplyLinkWrench')
    persistent = add(force, 'persistent')
    for key, value in [('entity_name', 'wood_block'), ('entity_type', 'model'),
                       ('force', f'0 0 {-MASS*9.81}')]:
        add(persistent, key, value)
    poses = add(model, 'plugin', filename='ignition-gazebo-pose-publisher-system',
                name='ignition::gazebo::systems::PosePublisher')
    for key, value in [('publish_model_pose', 'true'), ('publish_nested_model_pose', 'true'), ('publish_link_pose', 'true'),
                       ('use_pose_vector_msg', 'true'), ('update_frequency', 50)]:
        add(poses, key, value)
