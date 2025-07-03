"""Rigid hand body."""
import contextlib
import tempfile

import numpy as np
import itertools
from lxml import etree
from transforms3d.euler import mat2euler
from .math_utils import mat2pb, pb2mat
from .mesh_utils import filter_mesh, save_mesh_obj
from .urdf_utils import create_link, create_joint
from .os_utils import mkdir_p, join

__all__ = ('HandBody')

JOINT_FIXED = 0
JOINT_REVOLUTE = 1

JOINT_TYPE = {
    JOINT_FIXED: 'fixed',
    JOINT_REVOLUTE: 'revolute',
}


class HandBody:
    """Rigid multi-link hand body."""

    FLAG_STATIC = 1  # create static (fixed) body
    FLAG_ENABLE_COLLISION_SHAPES = 2  # enable colllision shapes
    FLAG_ENABLE_VISUAL_SHAPES = 4  # enable visual shapes
    FLAG_JOINT_LIMITS = 8  # apply joint limits
    FLAG_DYNAMICS = 16  # overide default dynamics parameters
    FLAG_USE_SELF_COLLISION = 32  # enable self collision
    FLAG_DEFAULT = sum([FLAG_ENABLE_COLLISION_SHAPES,
                        FLAG_ENABLE_VISUAL_SHAPES,
                        FLAG_JOINT_LIMITS,
                        FLAG_DYNAMICS,
                        FLAG_USE_SELF_COLLISION])

    def __init__(self, hand_model, flags=FLAG_DEFAULT, shape_betas=None,
                 urdf_dir="/tmp/mano_urdf",
                 urdf_object_format='stl'):
        """HandBody constructor.

        Arguments:
            client {PybulletClient} -- pybullet client
            hand_model {HandModel} -- rigid hand model

        Keyword Arguments:
            flags {int} -- configuration flags (default: {FLAG_DEFAULT})
            color {list} -- color RGBA (default: {None})
            shape_betas {array} -- MANO shape beta parameters  (default: {None})
        """
        self._model = hand_model
        self._flags = flags
        self._urdf_dir = urdf_dir
        self._urdf_object_format = urdf_object_format
        self._vertices = hand_model.vertices(betas=shape_betas)
        self._origin = self._model.origins()[0] 
        self._joint_indices = []
        self._joint_limits = []
        self._link_mapping = {}
        self._body_id = self._make_body()
        
    @ property
    def body_id(self):
        """Body unique id in the simulator.

        Returns:
            int -- body unique id in PyBullet
        """
        return self._body_id

    @ property
    def joint_indices(self):
        """Articulated joint indices.

        Returns:
            list -- list of joint indices
        """
        return self._joint_indices

    @ property
    def joint_limits(self):
        """Articulated joints angle bounds.

        Returns:
            list -- list of tuples (lower limit, upper limit)
        """
        return self._joint_limits

    def _make_body(self):
        joints = self._model.joints
        link_masses = [0.2]
        link_collision_indices = [self._make_collision_shape(0, joints[0].basis.T)]
        link_visual_indices = [self._make_visual_shape(0, joints[0].basis.T)]
        link_positions = [joints[0].origin]
        link_orientations = [mat2pb(joints[0].basis)]
        link_parent_indices = [0]
        link_joint_types = [JOINT_FIXED]
        link_joint_axis = [[0.0, 0.0, 0.0]]
        self._link_mapping[0] = 0

        for i, j in self._model.kintree_table.T[1:]:
            parent_index = self._link_mapping[i]
            origin_rel = joints[i].basis.T @ (joints[j].origin - joints[i].origin)
            basis_rel = joints[i].basis.T @ joints[j].basis

            for axis, limits in zip(joints[j].axes, joints[j].limits):
                link_masses.append(0.0)
                link_collision_indices.append(-1)
                link_visual_indices.append(-1)
                link_positions.append(origin_rel)
                link_orientations.append(mat2pb(basis_rel))
                link_parent_indices.append(parent_index+1)
                link_joint_types.append(JOINT_REVOLUTE)
                link_joint_axis.append(np.eye(3)[ord(axis) - ord('x')])
                origin_rel, basis_rel = [0.0, 0.0, 0.0], np.eye(3)
                parent_index = len(link_masses) - 1
                self._link_mapping[j] = parent_index
                self._joint_indices.append(parent_index)
                self._joint_limits.append(limits)
            
            link_masses[-1] = 0.02
            link_visual_indices[-1] = self._make_visual_shape(j, joints[j].basis.T)
            link_collision_indices[-1] = self._make_collision_shape(j, joints[j].basis.T)

        base_mass = 0.01
        if self.FLAG_STATIC & self._flags:
            base_mass = 0.0
        
        self._link_masses = link_masses
        self._link_collision_indices = link_collision_indices
        self._link_visual_indices = link_visual_indices
        self._link_positions=link_positions
        self._link_orientations=link_orientations
        self._link_parent_indices=link_parent_indices
        self._link_joint_types=link_joint_types
        self._link_joint_axis=link_joint_axis
        
    def create_urdf(self, prefix=''):
        """create a mano.urdf for the current mano hand under urdf_dir
    
        Parameters
        ----------
        prefix : str
            prefix for urdf mesh path.
    
        Returns
        -------
            path of the created urdf file or -1 if failed
        """
        joints = self._model.joints
        link_masses = [0.2]
        link_collision_indices = [self._make_collision_shape(0, joints[0].basis.T)]
        link_visual_indices = [self._make_visual_shape(0, joints[0].basis.T)]
        link_positions = [joints[0].origin]
        link_orientations = [mat2pb(joints[0].basis)]
        link_parent_indices = [0]
        link_joint_types = [JOINT_FIXED]
        link_joint_axis = [[0.0, 0.0, 0.0]]
        self._link_mapping[0] = 0
    
        # initiate urdf
        urdf_namespace = "http://www.ros.org/wiki/xacro"
        XHTML = "{%s}" % urdf_namespace
        NSMAP = {'xacro' : urdf_namespace} 
        robot = etree.Element("robot", name="mano_hand", nsmap=NSMAP)
        parent_link_name = 'root' 
    
        robot.append(create_link('root', prefix=prefix))
        robot.append(create_link('palm_x_link', prefix=prefix))
        robot.append(create_link('palm_y_link', prefix=prefix))
        robot.append(create_link('palm_z_link', prefix=prefix))
        robot.append(create_link('palm_roll_link', prefix=prefix))
        robot.append(create_link('palm_pitch_link', prefix=prefix))
        robot.append(create_link('palm_yaw_link', prefix=prefix))
        robot.append(create_link('palm_fixed_link', prefix=prefix))
        robot.append(create_link('palm', 
                                 visual_mesh=self._link_visual_indices[0],
                                 collision_mesh=self._link_collision_indices[0],
                                 prefix=prefix))
    
        robot.append(create_joint('root_joint_fixed', 'fixed',
                                  parent_link='root',
                                  # xyz=self._origin,
                                  child_link='palm_x_link'))
    
        robot.append(create_joint('palm_x_joint', 'prismatic',
                                  parent_link='palm_x_link',
                                  child_link='palm_y_link',
                                  limits=[-10, 10],
                                  axis=[1, 0, 0]))
        robot.append(create_joint('palm_y_joint', 'prismatic',
                                  parent_link='palm_y_link',
                                  child_link='palm_z_link',
                                  limits=[-10, 10],
                                  axis=[0, 1, 0]))
        
        robot.append(create_joint('palm_z_joint', 'prismatic',
                                  parent_link='palm_z_link',
                                  child_link='palm_yaw_link',
                                  limits=[-10, 10],
                                  axis=[0, 0, 1]))
        robot.append(create_joint('palm_yaw_joint', 'revolute',
                                  parent_link='palm_yaw_link',
                                  child_link='palm_pitch_link',
                                  limits=[-3.141593, -3.141593],
                                  axis=[0, 0, 1]))
        robot.append(create_joint('palm_pitch_joint', 'revolute',
                                  parent_link='palm_pitch_link',
                                  child_link='palm_roll_link',
                                  limits=[-3.141593, -3.141593],
                                  axis=[0, 1, 0]))
        robot.append(create_joint('palm_roll_joint', 'revolute',
                                  parent_link='palm_roll_link',
                                  child_link='palm_fixed_link',
                                  limits=[-3.141593, -3.141593],
                                  axis=[1, 0, 0]))
    
    
        robot.append(create_joint('palm_joint_fixed', 'fixed',
                                  xyz=joints[0].origin,
                                  rpy=mat2pb(joints[0].basis),
                                  parent_link='palm_fixed_link',
                                  child_link='palm'))
    
        for i, j in self._model.kintree_table.T[1:]:
            parent_index = self._link_mapping[i]
            parent_link_name = self._model.link_names[i]
            child_link_name = self._model.link_names[j]
            origin_rel = joints[i].basis.T @ (joints[j].origin - joints[i].origin)
            basis_rel = joints[i].basis.T @ joints[j].basis
    
            # axis_cnt = 0
            link_name = child_link_name
            num_axis = len(joints[j].axes)
            for axis_id, (axis, limits) in enumerate(zip(joints[j].axes, joints[j].limits)):
                joint_name = '{}_{}_joint'.format(child_link_name, axis)
                if axis_id != num_axis - 1:
                    # virtual link                    
                    link_name = '{}_{}'.format(child_link_name, axis)
                    link_node = create_link(link_name, prefix=prefix)                    
                    joint_node = create_joint(joint_name, JOINT_TYPE[JOINT_REVOLUTE], 
                        axis=np.eye(3)[ord(axis) - ord('x')], 
                        xyz=origin_rel,
                        rpy=mat2pb(basis_rel),
                        parent_link=parent_link_name,
                        child_link=link_name,
                        limits=limits)
                else: # last axis
                    link_node = create_link(child_link_name,
                                            visual_mesh=self._make_visual_shape(j, joints[j].basis.T),
                                            collision_mesh=self._make_collision_shape(j, joints[j].basis.T),
                                            prefix=prefix)
                    joint_node = create_joint(joint_name, JOINT_TYPE[JOINT_REVOLUTE], 
                        axis=np.eye(3)[ord(axis) - ord('x')], 
                        parent_link=parent_link_name,
                        child_link=child_link_name,
                        limits=limits)
                    
                origin_rel, basis_rel = [0.0, 0.0, 0.0], np.eye(3)
                robot.append(link_node)
                robot.append(joint_node)                    
                parent_link_name = link_name
    
        # write to urdf file    
        tree = etree.ElementTree(robot)
        mkdir_p(self._urdf_dir)
        filename = join(self._urdf_dir, 'mano.urdf')
        try:
            with open(filename, 'wb') as destination:
                tree.write(destination, pretty_print=True, xml_declaration=True, encoding='UTF-8')
            return filename
        except:
            return -1

    def _make_collision_shape(self, link_index, basis):
        if self.FLAG_ENABLE_COLLISION_SHAPES & self._flags:
            with self._temp_link_mesh(
                link_index, 
                True, 
                file_type=self._urdf_object_format) as filename:
                return filename
        return -1

    def _make_visual_shape(self, link_index, basis):
        if self.FLAG_ENABLE_VISUAL_SHAPES & self._flags:
            with self._temp_link_mesh(
                link_index, 
                False, 
                file_type=self._urdf_object_format) as filename:
                return filename
        return -1
    
    def set_target_from_mano(self, trans, mano_pose):
        """Set target hand state from a Mano pose.

        Arguments:
            mano_pose {array} -- pose of the Mano model
            trans {vec3} -- hand translation
        """
        angles, basis = self._model.mano_to_angles(mano_pose)
        trans = trans + self._origin - basis @ self._origin
        return trans, basis, angles
    
    def get_joint_config_from_mano(self, trans, mano_pose, side='left'):
        """get joint configurations from a mano pose.

        Arguments:
            mano_pose {array} -- pose of the Mano model
            trans {vec3} -- hand translation
        Return:
            Dictionary of joint configurations (joint name:value pairs)
        """
        trans, basis, angles = self.set_target_from_mano(trans, mano_pose)
        global_eulers = mat2euler(basis)
        
        configs = {}
        for id, joint_name in enumerate(['palm_x_joint', 'palm_y_joint', 'palm_z_joint']):
            configs[joint_name] = trans[id]
        
        configs['palm_roll_joint'] = global_eulers[0]
        configs['palm_pitch_joint'] = global_eulers[1]
        configs['palm_yaw_joint'] = global_eulers[2] 
        
        angle_id = 0
        for link, bone_id, axis in itertools.product(['index', 'middle', 'pinky', 'ring', 'thumb'], [1, 2, 3], 'xyz'):
            joint_name = '%s%d_%s_joint' % (link, bone_id, axis)
            configs[joint_name] = angles[angle_id]
            angle_id += 1
        return configs 

    @contextlib.contextmanager
    def _temp_link_mesh(self, link_index, collision, file_type='stl'):
        mesh_dir = join(self._urdf_dir, 'meshes')
        mkdir_p(mesh_dir)
        
        temp_file = tempfile.NamedTemporaryFile(suffix='.' + file_type)
        temp_file.close()
        threshold = 0.2
        link_name = self._model.link_names[link_index]
        if collision:            
            link_name += '_collision'
        if collision and link_index in [4, 7, 10]:
            threshold = 0.7
        vertex_mask = self._model.weights[:, link_index] > threshold
        vertices, faces = filter_mesh(self._vertices, self._model.faces, vertex_mask)
        vertices -= self._model.joints[link_index].origin
        file_path = join(mesh_dir, '{}.{}'.format(link_name, file_type))
        save_mesh_obj(file_path, vertices, faces, vhacd=True)
        yield join('meshes', '{}.{}'.format(link_name, file_type))
