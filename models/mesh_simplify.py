#!/usr/bin/env python
""" simpify collision meshes """
import sys
from math import pi
from glob import glob
import xml.etree.ElementTree as ET

import trimesh
import numpy as np
from numpy import triu_indices, triu_indices_from
from trimesh.transformations import \
        euler_from_matrix, \
        euler_matrix, \
        translation_from_matrix, \
        translation_matrix, \
        inverse_matrix, \
        concatenate_matrices


def transform2pose(transform):
    """convert 4x4 homogenous transform to a 6D pose"""
    return list(translation_from_matrix(transform)) + list(euler_from_matrix(transform))


def sym_to_mat(tri_array):
    """take an array containing triangular matrix elements and return a symmetric matrix"""
    n = int((2 * len(tri_array))**0.5)
    if len(tri_array) != (n * (n + 1)) // 2:
        print("Length of array unsuitable for creating a symmetric matrix!")
        return None
    symmetric = np.zeros((n, n))
    R, C = triu_indices(n)
    symmetric[R, C] = tri_array
    symmetric[C, R] = tri_array
    return symmetric


def inertia_from_xml(inode):
    """extract the inertia tensor from xml"""
    if inode.attrib:
        ixx = float(inode.attrib['ixx'])
        ixy = float(inode.attrib['ixy'])
        ixz = float(inode.attrib['ixz'])
        iyy = float(inode.attrib['iyy'])
        iyz = float(inode.attrib['iyz'])
        izz = float(inode.attrib['izz'])
    else:
        ixx = float(inode.find('ixx').text)
        ixy = float(inode.find('ixy').text)
        ixz = float(inode.find('ixz').text)
        iyy = float(inode.find('iyy').text)
        iyz = float(inode.find('iyz').text)
        izz = float(inode.find('izz').text)
    return sym_to_mat([ixx, ixy, ixz, iyy, iyz, izz])


def inertia_to_xml(itensor):
    """convert an inertia tensor to xml"""
    arr = np.array(itensor)
    ixx, ixy, ixz, iyy, iyz, izz = arr[triu_indices_from(arr)]
    return ET.fromstring(
        f"""
        <inertia>
        <ixx>{ixx}</ixx>
        <ixy>{ixy}</ixy>
        <ixz>{ixz}</ixz>
        <iyy>{iyy}</iyy>
        <iyz>{iyz}</iyz>
        <izz>{izz}</izz>
        </inertia>
        """
    )


def solid2xml(solid, x_r, y_l=None, z_z=None):
    """ convert simple solid to xml """
    if solid == 'sphere':
        return ET.fromstring(
            f"""
            <sphere>
            <radius>{x_r}</radius>
            </sphere>
            """
        )
    if solid == 'cylinder':
        return ET.fromstring(
            f"""
            <cylinder>
            <length>{y_l}</length>
            <radius>{x_r}</radius>
            </cylinder>
            """
        )
    if solid == 'box':
        return ET.fromstring(
            f"""
            <box>
            <size>{x_r} {y_l} {z_z}</size>
            </box>
            """
        )
    return None


def approximate_mesh(mesh_file):
    """find minimum volume bounding solid or approximate mesh given a complex mesh"""
    mesh_obj = trimesh.load(mesh_file, force='mesh')

    box = trimesh.bounds.oriented_bounds(mesh_obj)
    sphere = trimesh.nsphere.fit_nsphere(mesh_obj.vertices)
    cylinder = trimesh.bounds.minimum_cylinder(mesh_obj, sample_count=6)
    decomp = trimesh.util.concatenate(trimesh.decomposition.convex_decomposition(mesh_obj))

    vol_box = box[1][0] * box[1][1] * box[1][2]
    print('\tbox:', vol_box)
    vol_sphere = (4 / 3) * pi * (sphere[1]**2)
    print('\tshere:', vol_sphere)
    vol_cylinder = cylinder['height'] * pi * (cylinder['radius']**2)
    print('\tcylinder', vol_cylinder)
    vol_decomp = decomp.volume
    print('\tconvex:', vol_decomp)

    transforms = {}
    transforms[vol_box] = inverse_matrix(box[0])
    transforms[vol_sphere] = translation_matrix([p for p in sphere[0]])
    transforms[vol_cylinder] = cylinder['transform']
    transforms[vol_decomp] = np.eye(4)

    shapes = {}
    shapes[vol_box] = ('box', *box[1])
    shapes[vol_sphere] = ('sphere', sphere[1])
    shapes[vol_cylinder] = ('cylinder', cylinder['radius'], cylinder['height'])
    shapes[vol_decomp] = ('mesh', decomp)

    min_volume = min(vol_box, vol_sphere, vol_cylinder, vol_decomp)

    # return {'transform': transforms[vol_box], 'shape': shapes[vol_box]}
    # return {'transform': transforms[vol_sphere], 'shape': shapes[vol_sphere]}
    # return {'transform': transforms[vol_cylinder], 'shape': shapes[vol_cylinder]}
    # return {'transform': transforms[vol_decomp], 'shape': shapes[vol_decomp]}
    return {'transform': transforms[min_volume], 'shape': shapes[min_volume]}


def main(argv):
    for filename in glob(argv[1]):
        print(filename)
        tree = ET.parse(filename)
        link = tree.find('.//*[collision]')
        inertial = link.find('inertial')
        if inertial is None:
            print("\tSkipping massless object!")
            continue

        to_remove = set()
        for ch in link:
            if ch.tag != 'collision':
                continue

            name = ch.attrib.get('name', '')
            print('\t', name)
            if name.count('collision_'):
                num_left = len(tree.findall(".//collision")) - len(to_remove)
                if num_left > 1 and not ch.find('.//mesh'):
                    to_remove.add(ch)
                    print('\tMarked for Removal...')
                    continue
                print('\tRenaming to "collision"')
                ch.attrib['name'] = 'collision'

            pose = ch.find('pose')
            geom = ch.find('geometry')
            if pose is not None:
                print('\tPose:', pose.text)
            else:
                print('\tNo Pose? Appending indentiy...')
                pose = ET.Element('pose')
                pose.text = '0 0 0 0 0 0'
                ch.append(pose)
            if geom is not None:
                print('\tGeom:', list(geom))
            else:
                print('\tNo geometry?')
                return

            mesh = geom.find("mesh")
            if mesh is not None:
                mesh_uri = mesh.find('uri')
                meshfile = mesh_uri.text.replace('model://', '')

                # get approximating solid or mesh
                solid_desc = approximate_mesh(meshfile)
                shape = solid_desc['shape']
                transform = solid_desc['transform']

                # set inertia to link frame
                com_transform = np.eye(4)
                ipose = inertial.find('pose')
                if ipose is None:
                    ipose = ET.Element('pose')
                    inertial.append(ipose)
                else:
                    com_transform = translation_matrix([float(x) for x in ipose.text.split()[:3]])
                ipose.text = '0 0 0 0 0 0'

                moment_transform = np.eye(4)
                inertia = inertial.find('inertia')
                if inertia is None:
                    inertia = inertia_to_xml(np.eye(3))
                else:
                    iarr = inertia_from_xml(inertia)
                    # different convention?
                    # iarr[:3, :3] *= -1
                    # iarr[(0, 1, 2), (0, 1, 2)] *= -1

                    # find transformation from world to principal axes
                    inertia_scale, inertia_basis = trimesh.inertia.principal_axis(iarr)
                    e0, e1, e2 = inertia_basis
                    moment_transform[:3, 0] = e0
                    moment_transform[:3, 1] = e1
                    if np.allclose(np.cross(e0, e1), e2):
                        moment_transform[:3, 2] = e2
                    elif np.allclose(np.cross(e0, e1), -e2):
                        moment_transform[:3, 2] = -e2
                    else:
                        print("Adjust vector equality tolerance! Are these equal?:")
                        print(np.cross(e0, e1), np.cross(e1, e0), e2)
                        return

                    inertial.remove(inertia)
                    inertia = inertia_to_xml(np.diag(inertia_scale))
                inertial.append(inertia)

                # compute pose for collision and visual geometry
                inertia_transform = inverse_matrix(concatenate_matrices(com_transform, moment_transform))
                pose_visual = transform2pose(inertia_transform)
                pose_collision = transform2pose(concatenate_matrices(inertia_transform, transform))

                # set visual pose
                vis_geom = link.find('visual')
                vpose = link.find('pose')
                if vpose is None:
                    vpose = ET.Element('pose')
                    vis_geom.append(vpose)
                vpose.text = ' '.join([str(p) for p in pose_visual])

                # set collision pose
                pose.text = ' '.join([str(p) for p in pose_collision])

                # update geometry
                if shape[0] == 'mesh':
                    col_meshfile = '.'.join(meshfile.split('.')[:-1] + ['col', 'dae'])
                    print('\tNew collision mesh file: ', col_meshfile)
                    with open(col_meshfile, 'wb') as f:
                        # bits = trimesh.exchange.stl.export_stl(shape[1])
                        bits = trimesh.exchange.dae.export_collada(shape[1])
                        f.write(bits)
                    mesh_uri.text = 'model://' + col_meshfile
                else:
                    print('\tReplacing mesh with', shape)
                    geom.remove(mesh)
                    solid_xml = solid2xml(*shape)
                    geom.append(solid_xml)
                    if shape[0] == 'sphere':
                        pose.text = '0 0 0 0 0 0'
            else:
                print('\tNot a mesh.')

        for e in to_remove:
            link.remove(e)

        tree.write(filename)


if __name__ == '__main__':
    main(sys.argv)
