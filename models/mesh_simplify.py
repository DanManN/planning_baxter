#!/usr/bin/env python
""" simpify collision meshes """
import sys
from math import pi
from glob import glob
import xml.etree.ElementTree as ET

import trimesh
# import numpy as np
from trimesh.transformations import euler_from_matrix, translation_from_matrix, inverse_matrix


def transform2pose(transform):
    """convert 4x4 homogenous transform to a 6D pose"""
    return list(translation_from_matrix(transform)) + list(euler_from_matrix(transform))


def mesh2primitive_solid(mesh_file):
    """find minimum volume bounding simple solid for given mesh"""
    mesh_obj = trimesh.load(mesh_file, force='mesh')
    sphere = trimesh.nsphere.minimum_nsphere(mesh_obj)
    cylinder = trimesh.bounds.minimum_cylinder(mesh_obj, sample_count=6)
    box = trimesh.bounds.oriented_bounds(mesh_obj)

    vol_sphere = (4 / 3) * pi * (sphere[1]**2)
    vol_cylinder = cylinder['height'] * pi * (cylinder['radius']**2)
    vol_box = box[1][0] * box[1][1] * box[1][2]

    poses = {}
    # poses[vol_sphere] = list(sphere[0]) + [0, 0, 0]
    # poses[vol_cylinder] = transform2pose(cylinder['transform'])
    # poses[vol_box] = transform2pose(inverse_matrix(box[0]))
    poses[vol_sphere] = [-p for p in sphere[0]] + [0, 0, 0]
    poses[vol_cylinder] = transform2pose(inverse_matrix(cylinder['transform']))
    poses[vol_box] = transform2pose(box[0])

    shapes = {}
    shapes[vol_sphere] = ('sphere', sphere[1])
    shapes[vol_cylinder] = ('cylinder', cylinder['radius'], cylinder['height'])
    shapes[vol_box] = ('box', *box[1])

    min_volume = min(vol_sphere, vol_cylinder, vol_box)

    return {'pose': poses[min_volume], 'shape': shapes[min_volume]}


def primitive_solid2xml(solid, x_r, y_l=None, z_z=None):
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


if __name__ == '__main__':
    for filename in glob(sys.argv[1]):
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
                sys.exit(-1)

            mesh = geom.find("mesh")
            if mesh is not None:
                meshfile = mesh.find('uri').text.replace('model://', './')
                solid_desc = mesh2primitive_solid(meshfile)
                print('\tReplacing mesh with', solid_desc)
                geom.remove(mesh)
                solid_xml = primitive_solid2xml(*solid_desc['shape'])
                geom.append(solid_xml)
                # pose.text = ' '.join([str(p) for p in solid_desc['pose']])

                vis_geom = link.find('visual')
                vpose = link.find('pose')
                if vpose is None:
                    vpose = ET.Element('pose')
                    vis_geom.append(vpose)
                vpose.text = ' '.join([str(p) for p in solid_desc['pose']])

                ipose = inertial.find('pose')
                if ipose is None:
                    ipose = ET.Element('pose')
                    inertial.append(ipose)
                ipose.text = '0 0 0 0 0 0'
            else:
                print('\tNot a mesh.')

        for e in to_remove:
            link.remove(e)

        tree.write(filename)
