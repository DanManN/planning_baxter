# Node.py  2022-30-04
# MIT LICENSE 2022 Ewerton R. Vieira

from math import log1p, log2, log10, sqrt
from random import choice, uniform
import numpy as np
import time


class Node(object):
    def __init__(self, nodeID, current_config, path_region, radii, parent, depth, PH, Stick) -> None:
        super().__init__()
        self.nodeID = nodeID
        self.current_config = current_config
        self.path_region = path_region
        self.radii = radii  # all positives
        self.parent = parent
        self.depth = depth
        self.PH = PH
        self.Stick = Stick

        # self.isfeasible = True

        self.number_CC = PH.number_CC  # "Number of Connected Components of the current Path Region"

        """unvisited_directed_radii includes positives and negatives radius
        (negative means take other direction for pushing)"""

        if radii:
            self.unvisited_d_radii = set(radii) | set(-radius for radius in radii)
        else:
            self.unvisited_d_radii = set()
        self.children = dict()
        self.d_radius_from_parent = None
        self.action_from_parent = None
        self.visited_time = 0
        self.total_reward = 0
        self.iscomplete = False  # all children are feasible

    def UCB(self):
        c = 1.0
        assert len(
            self.unvisited_d_radii) == 0, f" \033[93m error: nonempty unvisited_d_radii \033[0m"
        Best_d_radius = False
        BestReward = - len(self.path_region)  # minimal reward
        for d_radius, child in self.children.items():
            if child.iscomplete:
                continue

            Reward = child.total_reward/child.visited_time + c * \
                sqrt(2*log10(self.visited_time)/child.visited_time)
            if Reward >= BestReward:
                BestReward = Reward
                Best_d_radius = d_radius

        if Best_d_radius == False:  # all children are complete
            self.iscomplete = True
            return self.parent
        else:
            return self.children[Best_d_radius]

    def expansion(self):
        d_radius = choice(list(self.unvisited_d_radii))
        self.unvisited_d_radii.remove(d_radius)
        return self.pushing_actions(d_radius)

    def number_obstacles(self):
        "Number of Obstacles in the current Path Region"
        return len(self.path_region)

    def pushing_actions(self, d_radius):
        # action here is the collection of pushing to clean a connected component
        """TODO: add the following to avoid useless actions:
        new_config == current_node.current_config for an epsilon and only obstacles"""
        # if new_config == current_node.current_config:
        #     # avoid to propagate action that haven't change anything
        #     new_radii = None

        # set configuration to the current_config to do the next pushing

        time.sleep(0.5)

        self.Stick.set_config(self.current_config)

        self.PH.world = self.Stick.world()  # update the world_positions

        self.PH.update()  # update all varibles that depends on the new positions

        time.sleep(0.5)

        sign = np.sign(d_radius)
        radius_ = abs(d_radius)
        square = self.PH.squared_CC(self.PH.path_region, self.PH.closest_pt, radius_)
        isfeasible = True

        if square:
            self.PH.push_planning(square, sign)
            print("IS FEASIBLE", self.Stick.is_feasible())
            isfeasible = self.Stick.is_feasible()
        else:
            self.PH.action_performed = None

        self.PH.world = self.Stick.world()  # update the world_positions

        self.PH.update()  # update all varibles that depends on the new positions

        time.sleep(0.5)

        if isfeasible:
            radii_ = self.PH.radii
            path_region_ = self.PH.path_region
        else:
            radii_ = set()  # if not feasible then dont compute radii
            path_region_ = set([-1, -1])  # if not feasible path_region is set to be nonempty

        return self.PH.action_performed, self.Stick.read_config(), path_region_, radii_, d_radius


# For PHIS


    def select_child(self):  # select the first available child
        assert len(
            self.unvisited_d_radii) == 0, f" \033[93m error: nonempty unvisited_d_radii \033[0m"
        selected_child = False
        for d_radius, child in self.children.items():
            if child.iscomplete:
                continue
            else:
                return child

        self.iscomplete = True
        return self.parent

    #
    #
    # def get_path_region(self):
    #     "assign path_region"
    #     return self.PH.path_region
    #
    # def get_config(self):
    #     "assign config"
    #     return self.Stick.read_config()
    #
    # def get_action(self):
    #     "assign action"
    #     return self.PH.action_performed
    #
    # def get_radii(self):
    #     "assign radii"
    #     return self.PH.radii
    #
    # def number_CC(self, radius):
    #     "Number of Connected Components of the current Path Region"
    #     return self.PH.number_CC(radius)
