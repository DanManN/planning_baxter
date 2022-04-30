# Node.py  2022-30-04
# MIT LICENSE 2022 Ewerton R. Vieira

from math import log1p, log2, log10, sqrt
from random import choice, uniform


class Node(object):
    def __init__(self, nodeID, current_config, path_region, radii, parent, PH, Stick) -> None:
        super().__init__()
        self.nodeID = nodeID
        self.current_config = current_config
        self.path_region = path_region
        if radii:
            self.unvisited_radii = set(radii)
        else:
            self.unvisited_radii = set()

        self.children = dict()
        self.parent = parent
        self.radius_from_parent = None
        self.action_from_parent = None
        self.visited_time = 0
        self.total_reward = 0
        self.radii = radii

        self.PH = PH
        self.Stick = Stick

    def UCB(self):
        c = 1.0
        assert len(self.unvisited_radii) == 0, f" error: nonempty unvisited_radii"
        Best_radius = 0
        BestReward = - len(self.path_region)  # minimal reward
        for radius, child in self.children.items():
            Reward = child.total_reward/child.visited_time + c * \
                sqrt(2*log10(self.visited_time)/child.visited_time)
            if Reward >= BestReward:
                BestReward = Reward
                Best_radius = radius
            else:
                print(f"\033[93m Wrong \033[93m {Reward} < {BestReward} \033[0m")
                Best_radius = radius
        return self.children[Best_radius]

    def expansion(self):
        radius = choice(list(self.unvisited_radii))
        self.unvisited_radii.remove(radius)
        return self.pushing_actions(radius)

    def pushing_actions(self, radius):
        # action here is the collection of pushing to clean a connected component
        """TODO: add the following to avoid useless actions:
        new_config == current_node.current_config for an epsilon and only obstacles"""
        # if new_config == current_node.current_config:
        #     # avoid to propagate action that haven't change anything
        #     new_radii = None

        # set configuration to the current_config to do the next pushing
        self.Stick.set_config(self.current_config)

        square = self.PH.squared_CC(self.PH.path_region, self.PH.closest_pt, radius)

        if square:
            self.PH.push_planning(square)
        else:
            self.PH.action_performed = None

        self.PH.world = self.Stick.world()  # update the world_positions

        self.PH.update()  # update all varibles that depends on the new positions

        return self.get_action(), self.get_config(), self.get_path_region(), self.get_radii(), radius

    def number_obstacles(self):
        "Number of Obstacles in the current Path Region"
        return len(self.path_region)

    def number_CC(self, radius):
        "Number of Connected Components of the current Path Region"
        return self.PH.number_CC(radius)

    def get_path_region(self):
        "assign path_region"
        return self.PH.path_region

    def get_config(self):
        "assign config"
        return self.Stick.read_config()

    def get_action(self):
        "assign action"
        return str(self.PH.action_performed)

    def get_radii(self):
        "assign radii"
        return self.PH.radii
