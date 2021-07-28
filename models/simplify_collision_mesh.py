import sys
from glob import glob
import xml.etree.ElementTree as ET


def mesh2primitive_solid(mesh_file):
    print(mesh_file)
    # return 'sphere', 10
    # return 'cylinder', 10, 20
    return 'box', 10, 20, 30


def primitive_solid(geo_type, x_r, y_l=None, z=None):
    if geo_type == 'sphere':
        return ET.fromstring(
            f"""
            <sphere>
            <radius>{x_r}</radius>
            </sphere>
            """
        )
    if geo_type == 'cylinder':
        return ET.fromstring(
            f"""
            <cylinder>
            <length>{y_l}</length>
            <radius>{x_r}</radius>
            </cylinder>
            """
        )
    if geo_type == 'box':
        return ET.fromstring(
            f"""
            <box>
            <size>{x_r} {y_l} {z}</size>
            </box>
            """
        )


if __name__ == '__main__':
    for filename in glob(sys.argv[1]):
        print(filename)
        tree = ET.parse(filename)
        link = tree.find(".//*[collision]")
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
                solid = primitive_solid(*solid_desc)
                geom.append(solid)
            else:
                print('\tNot a mesh.')

        for e in to_remove:
            link.remove(e)

        tree.write(filename)
