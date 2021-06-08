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

from moveit_commander import PlanningSceneInterface
from moveit_msgs.msg import PlanningScene, CollisionObject, AttachedCollisionObject
from planit.msg import PercievedObject
from geometry_msgs.msg import Pose, Point
from shape_msgs.msg import SolidPrimitive, Plane, Mesh, MeshTriangle
from moveit_commander.exception import MoveItCommanderException
from moveit_commander.conversions import *

try:
    from pyassimp import pyassimp
except:
    # support pyassimp > 3.0
    try:
        import pyassimp
    except:
        pyassimp = False
        print("Failed to import pyassimp, see https://github.com/ros-planning/moveit/issues/86 for more info")


class StreamedSceneInterface(PlanningSceneInterface):
    def __init__(self, ns="", synchronous=False, service_timeout=5.0):
        super().__init__(ns, synchronous, service_timeout)

    def updatePerception(self, msg):
        # ignore attached objects for now
        if msg.name in super().get_attached_objects():
            return True

        co = CollisionObject()
        co.id = msg.name
        co.header = msg.header
        if msg.type == PercievedObject.MESH:
            # print("MESH")
            co.meshes = [msg.mesh]
            co.mesh_poses = [msg.pose]
        elif msg.type == PercievedObject.SOLID_PRIMITIVE:
            # print("SOLID")
            solid = SolidPrimitive()
            if msg.solid.type == SolidPrimitive.BOX:
                solid.type = SolidPrimitive.BOX
            elif msg.solid.type == SolidPrimitive.SPHERE:
                solid.type = SolidPrimitive.SPHERE
            elif msg.solid.type == SolidPrimitive.CYLINDER:
                solid.type = SolidPrimitive.CYLINDER
            else:
                return False
            solid.dimensions = list(msg.solid.dimensions).copy()
            co.primitives = [solid]
            co.primitive_poses = [msg.pose]
        # elif msg.type == PercievedObject.POSE:
        else:
            return False

        if msg.name in super().get_known_object_names():
            co.operation = CollisionObject.MOVE
        else:
            co.operation = CollisionObject.APPEND
        self.add_object(co)
        return True

    def add_mesh(self, name, pose, mesh, size=(1, 1, 1)):
        """ Add a mesh to the planning scene """
        if type(mesh) is not Mesh:
            super().add_mesh(name, pose, mesh, size)
            return

        co = self.__mesh_from_msg(name, pose, mesh, size)
        self.__submit(co, attach=False)

    def attach_mesh(self, link, name, pose=None, mesh=None, size=(1, 1, 1), touch_links=[]):
        if mesh is not None and type(mesh) is not Mesh:
            super().attach_mesh(link, name, pose, mesh, size, touch_links)
            return

        aco = AttachedCollisionObject()
        if (pose is not None) and (mesh is not None):
            aco.object = self.__mesh_from_msg(name, pose, mesh, size)
        else:
            aco.object = self.__make_existing(name)
        aco.link_name = link
        aco.touch_links = [link]
        if len(touch_links) > 0:
            aco.touch_links = touch_links
        self.__submit(aco, attach=True)

    @staticmethod
    def __mesh_from_msg(header, name, pose, mesh):
        co = CollisionObject()
        co.id = name
        co.header = header
        co.meshes = [mesh]
        co.mesh_poses = [pose]
        return co
