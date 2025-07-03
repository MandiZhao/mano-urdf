"""Mesh manipulating uility functions."""

import numpy as np
import trimesh

__all__ = ('filter_mesh', 'save_mesh_obj')


def filter_mesh(vertices, faces, vertex_mask):
    """Get a submesh from a mesh using vertex mask.

    Arguments:
        vertices {array} -- whole mesh vertices
        faces {array} -- whole mesh faces
        vertex_mask {boolean array} -- vertex filter
    """
    index_map = np.cumsum(vertex_mask) - 1
    faces_mask = np.all(vertex_mask[faces], axis=1)
    vertices_sub = vertices[vertex_mask]
    faces_sub = index_map[faces[faces_mask]]
    return vertices_sub, faces_sub


def save_mesh_obj(filename, vertices, faces, vhacd=False):
    """Save a mesh as an obj file.

    Arguments:
        filename {str} -- output file name
        vertices {array} -- mesh vertices
        faces {array} -- mesh faces
    """
    mesh = trimesh.Trimesh(vertices=vertices, faces=faces)
    if vhacd:
        # TODO: add vhacd decomposition
        pass
    mesh.export(filename)