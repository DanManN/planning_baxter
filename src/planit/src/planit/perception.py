# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from moveit_commander import conversions
from moveit_msgs.msg import PlanningScene, CollisionObject, AttachedCollisionObject
from geometry_msgs.msg import Pose, Point
from shape_msgs.msg import SolidPrimitive, Plane, Mesh, MeshTriangle
from moveit_commander.exception import MoveItCommanderException

try:
    from pyassimp import pyassimp
except:
    # support pyassimp > 3.0
    try:
        import pyassimp
    except:
        pyassimp = False
        print("Failed to import pyassimp, see https://github.com/ros-planning/moveit/issues/86 for more info")


class PlanningSceneInterface(object):
    """
    Python interface for a C++ PlanningSceneInterface.
    Uses both C++ wrapped methods and scene manipulation topics
    to manipulate the PlanningScene managed by the PlanningSceneMonitor.
    See wrap_python_planning_scene_interface.cpp for the wrapped methods.
    """
    def __init__(self, ns="", synchronous=False, service_timeout=5.0):
        super().__init__(ns, synchronous, service_timeout)

    def add_mesh(self, name, pose, filename, size=(1, 1, 1)):
        """ Add a mesh to the planning scene """
        co = self.__make_mesh(name, pose, filename, size)
        self.__submit(co, attach=False)

    def attach_mesh(self, link, name, pose=None, filename="", size=(1, 1, 1), touch_links=[]):
        aco = AttachedCollisionObject()
        if (pose is not None) and filename:
            aco.object = self.__make_mesh(name, pose, filename, size)
        else:
            aco.object = self.__make_existing(name)
        aco.link_name = link
        aco.touch_links = [link]
        if len(touch_links) > 0:
            aco.touch_links = touch_links
        self.__submit(aco, attach=True)

    @staticmethod
    def __make_mesh(name, pose, filename, scale=(1, 1, 1)):
        co = CollisionObject()
        if pyassimp is False:
            raise MoveItCommanderException(
                "Pyassimp needs patch https://launchpadlibrarian.net/319496602/patchPyassim.txt"
            )
        scene = pyassimp.load(filename)
        if not scene.meshes or len(scene.meshes) == 0:
            raise MoveItCommanderException("There are no meshes in the file")
        if len(scene.meshes[0].faces) == 0:
            raise MoveItCommanderException("There are no faces in the mesh")
        co.operation = CollisionObject.ADD
        co.id = name
        co.header = pose.header

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
        co.meshes = [mesh]
        co.mesh_poses = [pose.pose]
        pyassimp.release(scene)
        return co
