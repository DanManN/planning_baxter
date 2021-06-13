#!/usr/bin/env python
from __future__ import print_function

import copy
import os.path
import argparse

import rospy
from rospkg import RosPack, ResourceNotFound

from tf.transformations import *
from geometry_msgs.msg import Point
from gazebo_msgs.msg import ModelStates
from shape_msgs.msg import SolidPrimitive, Mesh, MeshTriangle
from moveit_commander.conversions import *

import pysdf
from planit.msg import PercievedObject  #, PercieveObject

try:
    from pyassimp import pyassimp
except:
    # support pyassimp > 3.0
    try:
        import pyassimp
    except:
        pyassimp = False
        print("Failed to import pyassimp, see https://github.com/ros-planning/moveit/issues/86 for more info")

supported_geometry_types = ['mesh', 'cylinder', 'sphere', 'box']

worldsdf = None
model_cache = {}
perceptPub = None
updatePeriod = 0.5
lastUpdateTime = None
use_collision = False
submodelsToBeIgnored = []

gazebo_rospack = RosPack()


def makeMesh(filename, scale=(1, 1, 1)):
    if pyassimp is False:
        raise MoveItCommanderException("Pyassimp needs patch https://launchpadlibrarian.net/319496602/patchPyassim.txt")
    try:
        scene = pyassimp.load(filename)
    except Exception:
        print("Could not load file: ", filename, file=sys.stderr)
    if not scene.meshes or len(scene.meshes) == 0:
        raise MoveItCommanderException("There are no meshes in the file")
    if len(scene.meshes[0].faces) == 0:
        raise MoveItCommanderException("There are no faces in the mesh")

    mesh = Mesh()
    first_face = scene.meshes[0].faces[0]
    if hasattr(first_face, "__len__"):
        for face in scene.meshes[0].faces:
            if len(face) == 3:
                triangle = MeshTriangle()
                triangle.vertex_indices = [face[0], face[1], face[2]]
                mesh.triangles.append(triangle)
    elif hasattr(first_face, "indices"):
        for face in scene.meshes[0].faces:
            if len(face.indices) == 3:
                triangle = MeshTriangle()
                triangle.vertex_indices = [
                    face.indices[0],
                    face.indices[1],
                    face.indices[2],
                ]
                mesh.triangles.append(triangle)
    else:
        raise MoveItCommanderException("Unable to build triangles from mesh due to mesh object structure")
    for vertex in scene.meshes[0].vertices:
        point = Point()
        point.x = vertex[0] * scale[0]
        point.y = vertex[1] * scale[1]
        point.z = vertex[2] * scale[2]
        mesh.vertices.append(point)
    pyassimp.release(scene)
    return mesh


def link2obj_msg(link, full_linkname, model_pose, use_collision=False, lifetime=rospy.Duration(0)):
    linkpart = None
    if use_collision:
        linkparts = getattr(link, 'collisions')
    else:  # visual
        linkparts = getattr(link, 'visuals')

    msgs = []

    for num, linkpart in enumerate(linkparts):
        # print(linkpart)
        if not linkpart.geometry_type in supported_geometry_types:
            if linkpart.geometry_type:
                print(
                    "Element %s with geometry type %s not supported. Ignored." %
                    (full_linkname, linkpart.geometry_type)
                )
                return None

        obj_msg = PercievedObject()
        obj_msg.header.frame_id = 'world'  # pysdf.sdf2tfname(full_linkname)
        obj_msg.header.stamp = rospy.get_rostime()
        # obj_msg.name = pysdf.sdf2tfname(full_linkname) + str(num)
        obj_msg.name = pysdf.sdf2tfname(full_linkname) + linkpart.name
        # obj_msg.pose = pysdf.homogeneous2pose_msg(linkpart.pose)
        # relative = pose_to_list(pysdf.homogeneous2pose_msg(linkpart.pose))
        # total_pose = [x + y for x, y in zip(relative[:3], absolute[:3])] + absolute[3:]
        # absolute = pose_to_list(model_pose)
        # obj_msg.pose = list_to_pose(total_pose)
        relative = linkpart.pose
        absolute = pysdf.pose_msg2homogeneous(model_pose)
        total_pose = concatenate_matrices(absolute, relative)
        obj_msg.pose = pysdf.homogeneous2pose_msg(total_pose)
        # print("Rel: ", relative, "Abs: ", absolute, "Tot: ", total_pose, sep='\n')
        obj_msg.mesh = Mesh()
        obj_msg.solid = SolidPrimitive()
        obj_msg.solid.dimensions = [0]

        if linkpart.geometry_type == 'mesh':
            obj_msg.type = PercievedObject.MESH
            mesh_resource = None
            for models_path in pysdf.models_paths:
                resource = linkpart.geometry_data['uri'].replace('model://', models_path + '/')
                if os.path.isfile(resource):
                    mesh_resource = resource
                    break
            # support URDF-like resource paths starting with model://
            if not mesh_resource and linkpart.geometry_data['uri'].startswith('model://'):
                stripped_uri = linkpart.geometry_data['uri'].replace('model://', '')
                uri_parts = stripped_uri.split('/', 1)

                if len(uri_parts) == 2:
                    package_name = uri_parts[0]
                    try:
                        package_path = gazebo_rospack.get_path(package_name)
                        mesh_path = os.path.join(package_path, uri_parts[1])
                        if os.path.isfile(mesh_path):
                            mesh_resource = mesh_path
                    except ResourceNotFound as e:
                        pass

            if not mesh_resource:
                print('ERROR! could not find resource: %s' % linkpart.geometry_data['uri'])
                return None

            # scale = [float(val) for val in linkpart.geometry_data['scale'].split()]
            # print(obj_msg.name, scale, linkpart)
            scale = (0.001, 0.001, 0.001)
            obj_msg.mesh = makeMesh(mesh_resource, scale)
            obj_msg.solid.type = SolidPrimitive.SPHERE
            # print(linkpart)
        else:
            obj_msg.type = PercievedObject.SOLID_PRIMITIVE
            if linkpart.geometry_type == 'box':
                obj_msg.solid.type = SolidPrimitive.BOX
                scale = [float(val) for val in linkpart.geometry_data['size'].split()]
                # obj_msg.solid.BOX_X, obj_msg.solid.BOX_Y, obj_msg.solid.BOX_Z = scale
                obj_msg.solid.dimensions = scale
            elif linkpart.geometry_type == 'sphere':
                obj_msg.solid.type = SolidPrimitive.SPHERE
                radius = float(linkpart.geometry_data['radius'])
                obj_msg.solid.dimensions = [radius]
                # obj_msg.pose.position.z += radius
            elif linkpart.geometry_type == 'cylinder':
                obj_msg.solid.type = SolidPrimitive.CYLINDER
                # obj_msg.scale.x = obj_msg.scale.y = 2.0 * float(linkpart.geometry_data['radius'])
                # obj_msg.scale.z = float(linkpart.geometry_data['length'])
                height = float(linkpart.geometry_data['length'])
                radius = float(linkpart.geometry_data['radius'])
                obj_msg.solid.dimensions = [height, radius]

        # print(obj_msg)
        # print(obj_msg.solid.dimensions)
        msgs.append(obj_msg)
    return msgs


def publish_link_marker(link, full_linkname, **kwargs):
    full_linkinstancename = full_linkname
    if 'model_name' in kwargs and 'instance_name' in kwargs:
        full_linkinstancename = full_linkinstancename.replace(kwargs['model_name'], kwargs['instance_name'], 1)
    if 'model_pose' in kwargs:
        model_pose = kwargs['model_pose']
    else:
        model_pose = list_to_pose([0, 0, 0, 0, 0, 0])

    # print(link, full_linkname, full_linkinstancename, kwargs)
    obj_msgs = link2obj_msg(link, full_linkinstancename, model_pose, use_collision, rospy.Duration(2 * updatePeriod))
    # print(obj_msgs)
    if len(obj_msgs) > 0:
        for obj_msg in obj_msgs:
            perceptPub.publish(obj_msg)


def on_model_states_msg(model_states_msg):
    global lastUpdateTime
    sinceLastUpdateDuration = rospy.get_rostime() - lastUpdateTime
    if sinceLastUpdateDuration.to_sec() < updatePeriod:
        return
    lastUpdateTime = rospy.get_rostime()

    for (modelinstance_name, model_pose) in zip(model_states_msg.name, model_states_msg.pose):
        model_name = pysdf.name2modelname(modelinstance_name)
        if not model_name in model_cache:
            model_cache[model_name] = None
            if worldsdf:
                for model in worldsdf.world.models:
                    if model.name == model_name:
                        model_cache[model_name] = model
                        break
            else:
                sdf = pysdf.SDF(model=model_name)
                if len(sdf.world.models) >= 1:
                    model_cache[model_name] = sdf.world.models[0]
            if model_cache[model_name]:
                rospy.loginfo('Loaded model: %s' % model_cache[model_name].name)
            else:
                rospy.loginfo('Unable to load model: %s' % model_name)
        model = model_cache[model_name]
        if not model:  # Not an SDF model
            pass
        else:
            model.for_all_links(
                publish_link_marker, model_name=model_name, instance_name=modelinstance_name, model_pose=model_pose
            )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-f', '--freq', type=float, default=2, help='Frequency Markers are published (default: 2 Hz)')
    parser.add_argument('-c', '--collision', action='store_true', help='Publish collision instead of visual elements')
    parser.add_argument('-w', '--worldfile', type=str, help='Read models from this world file')
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node('gazeboPerception')

    global submodelsToBeIgnored
    submodelsToBeIgnored = rospy.get_param('~ignore_submodels_of', '').split(';')
    rospy.loginfo('Ignoring submodels of: ' + str(submodelsToBeIgnored))
    if submodelsToBeIgnored:
        rospy.logerr('ignore_submodels_of is currently not supported and will thus have no effect')

    global updatePeriod
    updatePeriod = 1. / args.freq

    global use_collision
    use_collision = args.collision

    if args.worldfile:
        global worldsdf
        worldsdf = pysdf.SDF(file=args.worldfile)

    global perceptPub
    perceptPub = rospy.Publisher('/perception', PercievedObject, queue_size=10)
    # perceptSrv = rospy.Service('gazebo_perception', PercieveObject, add_two_ints)
    rospy.sleep(rospy.Duration(0, 100 * 1000))

    global lastUpdateTime
    lastUpdateTime = rospy.get_rostime()
    modelStatesSub = rospy.Subscriber('/gazebo/model_states', ModelStates, on_model_states_msg)
    # msg = rospy.wait_for_message('/gazebo/model_states', ModelStates)
    # on_model_states_msg(msg)

    rospy.loginfo('Spinning')
    rospy.spin()


if __name__ == '__main__':
    main()
