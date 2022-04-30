from random import choice

from Node import Node


class MCTS(object):
    def __init__(self, source_config, path_region, radii, PH, Stick) -> None:
        self.source_config = source_config
        self.path_region = path_region
        self.radii = radii

        self.PH = PH
        self.Stick = Stick
        self.root = Node(0, source_config, path_region, self.radii, None, self.PH, self.Stick)


        # outputs
        self.action_list = []
        self.isfeasible = False

        Max_iter = 1e6
        num_iter = 0
        while (num_iter < Max_iter):
            num_iter += 1
            current_node = self.selection()
            action, new_config, new_path_region, new_radii, radius = current_node.expansion()

            new_node = Node(num_iter, new_config, new_path_region,
                            new_radii, current_node, self.PH, self.Stick)

            new_node.radius_from_parent = radius
            new_node.action_from_parent = action

            current_node.children[radius] = new_node
            reward = self.reward_detection(new_node)
            self.back_propagation(new_node, reward)
            if new_path_region == []:
                self.isfeasible = True
                self.construct_plan(new_node)
                break
        print(num_iter)

    def construct_plan(self, node: Node):
        self.action_list = []
        current_node = node
        while current_node.parent != None:
            parent_node = current_node.parent
            action = current_node.action_from_parent

            self.action_list.append(action)
            current_node = parent_node
        self.action_list.reverse()

    def selection(self):
        current_node = self.root
        while len(current_node.unvisited_radii) == 0:
            current_node = current_node.UCB()

        return current_node

    def reward_detection(self, node):
        """reward removal of obstacles and creation of more Connected Components.
        Weight gives more relevance to either removal of obstacles or creation
        of connected components"""
        weight = 0
        dif_obstacles = node.parent.number_obstacles() - node.number_obstacles()
        dif_CC = 1 + node.number_CC(node.radius_from_parent) - node.parent.number_CC(node.radius_from_parent)
        if dif_CC == 0:
            dif_CC = -1
        return dif_obstacles + weight * dif_CC

    def back_propagation(self, node, reward):
        current_node = node
        while current_node != None:
            current_node.visited_time += 1
            current_node.total_reward += reward
            current_node = current_node.parent
