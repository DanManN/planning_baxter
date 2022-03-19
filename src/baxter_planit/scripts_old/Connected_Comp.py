# Connected_Comp.py

import numpy as np
import matplotlib.pyplot as plt


def s_neighbors(x, s):  # return all componets that are points in a s ball.
    #     x = x.tolist()
    # x0 = np.array([x[0, :]])
    comp_con = dict({0: {0}})  # pre_components
    flag = False
    CC = dict()  # connected components

    # building pre_components
    for i in range(len(x)):
        temp = set()
        for j in range(i, len(x)):
            xi = np.array([x[i, :]])
            xj = np.array([x[j, :]])
            if np.linalg.norm(xi - xj) <= s:
                temp.update({i, j})

        for k in comp_con.keys():
            if temp & comp_con.get(k):
                comp_con.update({k: comp_con.get(k) | temp})
                flag = True
                continue
        if not flag:
            comp_con.update({k + 1: temp})
            """creating a new pre_component
            (below it is checking if there is intersection with others pre_components"""
        flag = False

    # if the intersection of two pre_component is true then join both pre_component
#     print(comp_con)
    skip = []
    for i in range(len(comp_con.keys())):
        #         print(i)
        for j in range(i, len(comp_con.keys())):

            if j in skip:  # skip if j already in a component
                continue

            # nontrivial intesection update with union
            if comp_con.get(i) & comp_con.get(j):

                if not CC.get(i):  # creating first key i with union
                    CC.update({i: comp_con.get(i) | comp_con.get(j)})

                else:  # if key i exists then update CC with union
                    CC.update({i: CC.get(i) | comp_con.get(j)})

                # j in skip since one doesnt need to make this union again
                skip.append(j)
#                 print({i : comp_con.get(i) | comp_con.get(j)})

    return CC


def persistent_CC_r(x, diagram_0, r):
    """show all connected components that persists with r distance
    x is the data, diagram_0 is the persistent diagram for H_0, r is the distance"""
    CC_r = []
    # -2 since we dont need about infinity

    radius_diagram = []

    for i in range(len(diagram_0[:, 1]) - 1):
        if diagram_0[i + 1][1] - diagram_0[i][1] > r:
            # adding 0.0001 to pick s_neighbors greater than diagram_0[i][1]
            CC_r.append(s_neighbors(x, diagram_0[i][1] + 0.0001))

            radius_diagram.append(diagram_0[i][1])
            print(diagram_0[i][1])

    return CC_r, radius_diagram


def plot_CC(data, A):
    """Plot Connected Components
    A = Connected Components
    """
    markers = {'.': 'point', ',': 'pixel', 'o': 'circle', 'v': 'triangle_down',
    '^': 'triangle_up', '<': 'triangle_left', '>': 'triangle_right', '1': 'tri_down', '2': 'tri_up', '3': 'tri_left', '4': 'tri_right', '8': 'octagon', 's': 'square', 'p': 'pentagon', '*': 'star', 'h': 'hexagon1', 'H': 'hexagon2', '+': 'plus', 'x': 'x', 'D': 'diamond',
               'd': 'thin_diamond', '|': 'vline', '_': 'hline', 'P': 'plus_filled',
               'X': 'x_filled', 0: 'tickleft', 1: 'tickright', 2: 'tickup', 3: 'tickdown', 4: 'caretleft', 5: 'caretright', 6: 'caretup', 7: 'caretdown', 8: 'caretleftbase', 9: 'caretrightbase', 10: 'caretupbase', 11: 'caretdownbase', 'None': 'nothing', None: 'nothing', ' ': 'nothing', '': 'nothing'}
    colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd',
              '#8c564b', '#e377c2', '#7f7f7f', '#bcbd22', '#17becf']

    for i in A.keys():
        for j in A.get(i):
            plt.scatter(data[j, 0], data[j, 1], color=colors[i % 10],
                        marker=list(markers.keys())[i], s=100)
    plt.axis('equal')
    plt.show()


def plot_CC_radius(data, A, RADIUS):
    """Plot Connected Components
    A = Connected Components
    RADIUS = radius of the balls centered at each point
    """

    markers = {'.': 'point', ',': 'pixel', 'o': 'circle', 'v': 'triangle_down',
    '^': 'triangle_up', '<': 'triangle_left', '>': 'triangle_right',
    '1': 'tri_down', '2': 'tri_up', '3': 'tri_left', '4': 'tri_right',
    '8': 'octagon', 's': 'square', 'p': 'pentagon', '*': 'star',
    'h': 'hexagon1', 'H': 'hexagon2', '+': 'plus', 'x': 'x', 'D': 'diamond',
               'd': 'thin_diamond', '|': 'vline', '_': 'hline',
               'P': 'plus_filled', 'X': 'x_filled', 0: 'tickleft', 1: 'tickright',
               2: 'tickup', 3: 'tickdown', 4: 'caretleft', 5: 'caretright', 6: 'caretup',
               7: 'caretdown', 8: 'caretleftbase', 9: 'caretrightbase',
               10: 'caretupbase', 11: 'caretdownbase', 'None': 'nothing',
               None: 'nothing', ' ': 'nothing', '': 'nothing'}
    colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd',
              '#8c564b', '#e377c2', '#7f7f7f', '#bcbd22', '#17becf']

    # points_whole_ax = 72  # 0.5 * 0.8 * 72    # 1 point = dpi / 72 pixels

    # points_radius = 2 * RADIUS / 1.0 * points_whole_ax

    # for i in A.keys():
    #     for j in A.get(i):
    #         plt.scatter(data[j, 0], data[j, 1], s=points_radius **
    #                     2,  color=colors[i % 10], alpha=0.5)

    # plt.axis('equal')
    # plt.show()

    min_x = np.min(data[:, 0])
    max_x = np.max(data[:, 0])
    x_diff = max_x - min_x

    min_y = np.min(data[:, 1])
    max_y = np.max(data[:, 1])
    y_diff = max_y - min_y

    if max_x - min_x < max_y - min_y:
        ylim_max = max_y
        ylim_min = min_y

        xlim_min = - y_diff / 2 + (min_x + max_x) / 2
        xlim_max = y_diff / 2 + (min_x + max_x) / 2
    else:
        xlim_max = max_x
        xlim_min = min_x

        ylim_min = - x_diff / 2 + (min_y + max_y) / 2
        ylim_max = x_diff / 2 + (min_y + max_y) / 2

    EXTRA = 0.1

    xlim_max = xlim_max + EXTRA
    xlim_min = xlim_min - EXTRA

    ylim_min = ylim_min - EXTRA
    ylim_max = ylim_max + EXTRA

    # print(xlim_min, xlim_max, )

    RADIUS = RADIUS / 2  # it is inputing diameter

    FIG_SIZE = 2

    FIG_AXES_SIZE_MIN = 0

    FIG_AXES_SIZE_MAX = 2

    # RADIUS = 1

    # plt.figure(figsize=[FIG_SIZE, FIG_SIZE])
    # ax = plt.axes([FIG_AXES_SIZE_MIN, FIG_AXES_SIZE_MIN, FIG_AXES_SIZE_MAX, FIG_AXES_SIZE_MAX],
    #               xlim=(AXES_SIZE_MIN, AXES_SIZE_MAX), ylim=(AXES_SIZE_MIN, AXES_SIZE_MAX))

    plt.figure(figsize=[FIG_SIZE, FIG_SIZE])
    ax = plt.axes([FIG_AXES_SIZE_MIN, FIG_AXES_SIZE_MIN, FIG_AXES_SIZE_MAX, FIG_AXES_SIZE_MAX],
                  xlim=(xlim_min, xlim_max), ylim=(ylim_min, ylim_max))

    fig_axes_diff = FIG_AXES_SIZE_MAX - FIG_AXES_SIZE_MIN

    # axes_size_diff = AXES_SIZE_MAX - AXES_SIZE_MIN

    axes_size_diff = xlim_max - xlim_min

    points_whole_ax = FIG_SIZE * fig_axes_diff * 72    # 1 point = dpi / 72 pixels

    points_radius = 2 * RADIUS / axes_size_diff * points_whole_ax

    for i in A.keys():
        for j in A.get(i):
            ax.scatter(data[j, 0], data[j, 1], s=points_radius **
                       2,  color=colors[i % 10], alpha=0.2)
            ax.scatter(data[j, 0], data[j, 1], color=colors[i % 10],
                       marker=list(markers.keys())[i], s=100)

    # plt.grid()
    # plt.axis('equal')

    plt.show()

#delete me
# def rotate_plane(matrix):
#     """function need to rotate plane so data will be similar to Gazebo, then update matrix with this rotation"""
#
#     for j in range(len(matrix[:, 0])):
#         Mj_x = matrix[j, 1]
#         Mj_y = -matrix[j, 0]
#         matrix[j, :] = Mj_x, Mj_y
#
#     return matrix
