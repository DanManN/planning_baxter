import numpy as np
from scipy.spatial import KDTree
import matplotlib.pyplot as plt


class ICP(object):
    def __init__(self, src_pc, tgt_pc) -> None:
        self.RT = np.identity(2)
        self.t = np.zeros(2)
        self.src_pc = self.smaller_pc(src_pc)
        self.tgt_pc = self.smaller_pc(tgt_pc)
        # for i in range(self.src_pc.shape[0]):
        #     print("self.src_pc", self.src_pc[i, :])
        # for i in range(self.tgt_pc.shape[0]):
        #     print("self.tgt_pc", self.tgt_pc[i, :])
        print("original")
        plt.scatter(self.tgt_pc[:, 0], self.tgt_pc[:, 1], color="r")
        plt.scatter(self.src_pc[:, 0], self.src_pc[:, 1], color="b")
        # plt.pause(0.05)
        plt.show()
        # normalization
        src_center = np.array(
            [
                np.average(self.src_pc[:, 0]),
                np.average(self.src_pc[:, 1]),
            ]
        )

        tgt_center = np.array(
            [
                np.average(self.tgt_pc[:, 0]),
                np.average(self.tgt_pc[:, 1]),
            ]
        )

        self.t = tgt_center - src_center
        self.src_pc = self.src_pc + self.t

        self.T = KDTree(self.tgt_pc)
        self.ICP()
        self.src_pc = np.dot(src_pc, self.RT) + self.t
        # plt.cla()
        print("ICP")
        # plt.scatter(self.tgt_pc[:, 0], self.tgt_pc[:, 1], color="r")
        # plt.scatter(self.src_pc[:, 0], self.src_pc[:, 1], color="b")
        # # plt.pause(0.05)
        # plt.show()

    def smaller_pc(self, pc):
        smaller_pc = pc.astype(int)
        smaller_pc = np.unique(smaller_pc, axis=0)
        smaller_pc = smaller_pc.astype(float)
        print("smaller_pc", smaller_pc.shape)
        return smaller_pc

    def ICP(self):
        old_error = 100000000.0
        error = old_error / 100.0
        while (old_error - error) / old_error >= 0.0001:
            # plt.cla()
            # plt.scatter(self.tgt_pc[:, 0], self.tgt_pc[:, 1], color="r")
            # plt.scatter(self.src_pc[:, 0], self.src_pc[:, 1], color="b")
            # plt.pause(0.05)
            # plt.show(block=False)
            NN_pc = self.closest_points()
            # normalization
            src_center = np.array(
                [
                    np.average(self.src_pc[:, 0]),
                    np.average(self.src_pc[:, 1]),
                ]
            )

            tgt_center = np.array(
                [
                    np.average(NN_pc[:, 0]),
                    np.average(NN_pc[:, 1]),
                ]
            )

            norm_src = self.src_pc - src_center
            norm_tgt = NN_pc - tgt_center

            W = np.dot(np.transpose(norm_tgt), norm_src)

            u, s, vh = np.linalg.svd(W)
            # print(u)
            # print(s)
            # print(vh)
            R = np.dot(u, vh)
            # print("R", R)
            t = tgt_center - np.dot(R, src_center)
            # print("t", t)
            self.src_pc = np.transpose(np.dot(R, np.transpose(norm_src))) + tgt_center
            self.RT = np.dot(self.RT, np.transpose(R))
            self.t += t
            print("R", self.RT)
            print("t", self.t)
            old_error = error
            error = np.linalg.norm(self.src_pc - NN_pc)
            print("error", error)

        # plt.cla()
        # plt.scatter(self.tgt_pc[:, 0], self.tgt_pc[:, 1], color="r")
        # plt.scatter(self.src_pc[:, 0], self.src_pc[:, 1], color="b")
        # # plt.pause(0.05)
        # plt.show()
        # return total_RT, total_t

    def closest_points(self):
        Dist = self.T.query(self.src_pc)
        NN = self.T.data[Dist[1]]
        # print(NN)
        return NN
