from lxml import etree

def create_link(name, xyz=[0, 0, 0], rpy=[0, 0, 0], visual_mesh=None, collision_mesh=None, prefix=""):
    """Greate a link node.

    Parameters
    ----------
    name : str
        name of the link.
    visual_mesh : str
        path of the `visual_mesh`.
    collision_mesh : str
        path of the `collision_mesh`.

    Returns
    -------
    etree.Element
        a link node.
    """
    link = etree.Element("link", name=name)
    
    if visual_mesh is not None:
        visual = etree.SubElement(link, "visual")
        origin = etree.SubElement(visual, "origin", 
            xyz='%.6f %.6f %.6f' % (xyz[0], xyz[1], xyz[2]), 
            rpy='%.6f %.6f %.6f' % (rpy[0], rpy[1], rpy[2]))
        geometry = etree.SubElement(visual, "geometry")
        mesh = etree.SubElement(geometry, "mesh", filename=prefix+visual_mesh)
        
    if collision_mesh is not None:
        collision = etree.SubElement(link, "collision")
        origin = etree.SubElement(collision, "origin", 
            xyz='%.6f %.6f %.6f' % (xyz[0], xyz[1], xyz[2]), 
            rpy='%.6f %.6f %.6f' % (rpy[0], rpy[1], rpy[2]))
        geometry = etree.SubElement(collision, "geometry")
        mesh = etree.SubElement(geometry, "mesh", filename=prefix+collision_mesh)
    return link


def create_joint(name, type, axis=[0, 0, 0], xyz=[0, 0, 0], rpy=[0, 0, 0], 
    parent_link="", child_link="", limits=None):
    """Greate a joint node.

    Parameters
    ----------
    name : str
        name of the joint
    type : str
        joint type.
    axis : list
        joint axis.
    xyz : list
        origin translation of the joint.
    rpy : list
        origin rotation of the joint.
    parent_link : str
        parent link name.
    child_link : str
        child link name.
    limits : list[2]
        joint limits [lower, upper].

    Returns
    -------
    etree.Element
        a joint node.

    """
    joint = etree.Element("joint", name=name, type=type)
    axis = etree.SubElement(joint, "axis", xyz='%d %d %d' % (axis[0], axis[1], axis[2]))
    origin = etree.SubElement(joint, "origin", 
        xyz='%.6f %.6f %.6f' % (xyz[0], xyz[1], xyz[2]), 
        rpy='%.6f %.6f %.6f' % (rpy[0], rpy[1], rpy[2]))
    parent = etree.SubElement(joint, "parent", link=parent_link)
    child = etree.SubElement(joint, "child", link=child_link)
    if limits is not None:        
        limits = etree.SubElement(joint, "limit", effort='0.35', 
            lower='%.6f'%limits[0], upper='%.6f'%limits[1], velocity='10')
    return joint