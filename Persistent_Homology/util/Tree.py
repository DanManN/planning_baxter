# MCTS.py  2022-30-04
# MIT LICENSE 2022 Ewerton R. Vieira

from random import choice

from .Node import Node


class Tree(object):
    def __init__(self, source_config, path_region, radii, PH, Stick) -> None:
        self.source_config = source_config
        self.path_region = path_region
        self.radii = radii

        self.PH = PH
        self.Stick = Stick
        self.root = Node(0, source_config, path_region, self.radii, None, 0, self.PH, self.Stick)

        # outputs
        self.action_list = []
        self.isfeasible = False

        Max_iter = 50#1e6
        num_iter = 0

        if path_region:
            best_node = None  # node that will lead to the less actions
        else:
            best_node = self.root

        while (num_iter < Max_iter):
            num_iter += 1
            current_node = self.selection()
            if not current_node:  # break if it selects parent of root (happens when all nodes are completed)
                print("\033[93m all nodes explored and expanded\033[0m")
                break
            action, new_config, new_path_region, new_radii, radius = current_node.expansion()

            print(f"\033[96m node {current_node.nodeID}: visited_radii = {radius} \033[0m")

            if new_config == current_node.current_config:
                print("same config")

            new_node = Node(num_iter, new_config, new_path_region,
                            new_radii, current_node, current_node.depth + 1, self.PH, self.Stick)

            print(f"\033[95m node {current_node.nodeID} -> node {new_node.nodeID} \033[0m by visited_radii = {radius}")

            new_node.radius_from_parent = radius
            new_node.action_from_parent = action

            current_node.children[radius] = new_node

            # reward = self.reward_detection(new_node)
            # self.back_propagation(new_node, reward)

            if new_path_region == []:
                print("\033[92m solution found \033[0m")
                new_node.iscomplete = True
                if not best_node:
                    best_node = new_node

                if new_node.depth < best_node.depth:
                    print("\033[92m update solution  \033[0m")
                    best_node = new_node
                    if new_node.depth == 1:  # shortest solution
                        break

        print(num_iter)

        if best_node:  # fix
            self.isfeasible = True
            self.construct_plan(best_node)
        else:
            print(f"no solution found after {num_iter} iterations")

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
            current_node = current_node.select_child()
            if not current_node:  # will return None if there is no more options
                break
        if current_node:
            print(f"node {current_node.nodeID}: unvisited_radii = {current_node.unvisited_radii} / length {len(current_node.unvisited_radii)}")
        return current_node

    # def reward_detection(self, node):
    #     """reward removal of obstacles and creation of more Connected Components.
    #     Weight gives more relevance to either removal of obstacles or creation
    #     of connected components"""
    #     weight = 0
    #     dif_obstacles = node.parent.number_obstacles() - node.number_obstacles()
    #     dif_CC = 1 + node.number_CC(node.radius_from_parent) - \
    #         node.parent.number_CC(node.radius_from_parent)
    #     if dif_CC == 0:
    #         dif_CC = -1
    #     return dif_obstacles + weight * dif_CC

    # def back_propagation(self, node, reward):
    #     current_node = node
    #     while current_node != None:
    #         current_node.visited_time += 1
    #         current_node.total_reward += reward
    #         current_node = current_node.parent
