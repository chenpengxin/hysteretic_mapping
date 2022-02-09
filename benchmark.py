
import os
import numpy as np

import matplotlib
import matplotlib.pyplot as plt

import scipy
import scipy.stats

import open3d as o3d


class Benchmark(object):

    def __init__(self):
        self.METHODS = ["LOAM", "BALM", "Mapper", "MapperFeatureEqualizer"]

        self.REMOVE_SYSTEM_ERROR = True
        self.PRJ_PATH = os.path.dirname(os.path.realpath(__file__))
        # matplotlib settings
        fontsize = 60
        font = {'weight': 'normal',
                'size': fontsize}
        matplotlib.rc('font', **font)
        self.fontsize = fontsize


    """
    @API:  RIDE
    @Desc: Relative inter-target distance error
    """
    def RIDE(self):
        for method in self.METHODS:
            self.target2targetDistanceError(method)


    """
    @API:  ATPE
    @Desc: Absolute target positioning error
    """
    def ATPE(self):
        for method in self.METHODS:
            self.targetPositioningError(method)



    def plot_error(self, error):
        fig = plt.figure(figsize=(15, 15))
        ax = fig.add_subplot(111, projection='3d')
        x = y = np.arange(error.shape[0])
        X, Y = np.meshgrid(x, y)
        ax.plot_surface(X, Y, error, rstride=1, cstride=1, cmap=plt.get_cmap('rainbow'))
        plt.show()
        return



    def plot_targets(self, targets_tls, targets_mls):
        pcd_tls = o3d.geometry.PointCloud()
        pcd_tls.points = o3d.utility.Vector3dVector(targets_tls - np.mean(targets_tls, axis=0))
        colors = [[0, 0, 0] for i in range(targets_tls.shape[0])]
        pcd_tls.colors = o3d.utility.Vector3dVector(colors)

        pcd_mls = o3d.geometry.PointCloud()
        pcd_mls.points = o3d.utility.Vector3dVector(targets_mls - np.mean(targets_mls, axis=0))
        colors = [[1, 0, 0] for i in range(targets_mls.shape[0])]
        pcd_mls.colors = o3d.utility.Vector3dVector(colors)

        o3d.visualization.draw_geometries([pcd_tls] + [pcd_mls])



    def remove_system_error(self, errs):
        mu, std = scipy.stats.norm.fit(errs)
        errs = errs - mu
        return errs



    def target2targetDistanceError(self, method):
        targets_tls_path = os.path.join(self.PRJ_PATH, "TLS_groudtruth/tls_target.csv")
        targets_mls_path = os.path.join(self.PRJ_PATH, "MLS_sample_results", method, "target.csv")

        # 1) read target coordinates
        targets_tls_all = np.genfromtxt(targets_tls_path, delimiter=',', skip_header=1)
        targets_mls_all = np.genfromtxt(targets_mls_path, delimiter=',', skip_header=1)
        # print(targets_tls_all.shape, targets_mls_all.shape)
        valid_ids = targets_mls_all[:, 0]
        valid_mls = targets_mls_all[:, 1:]
        valid_tls = []
        for index in valid_ids:
            for target in targets_tls_all:
                if round(target[0]) == round(index):
                    valid_tls.append(target[1:])
                    break
        valid_tls = np.asarray(valid_tls)
        # plot_targets(valid_tls, valid_mls)

        # 2) compute relative error
        pdist_tls = (valid_tls.reshape(1, -1, 3) - valid_tls.reshape(-1, 1, 3)) ** 2
        pdist_tls = np.sqrt(pdist_tls.sum(-1))
        pdist_mls = (valid_mls.reshape(1, -1, 3) - valid_mls.reshape(-1, 1, 3)) ** 2
        pdist_mls = np.sqrt(pdist_mls.sum(-1))
        error = pdist_mls - pdist_tls
        # plot_error(error)
        # print(np.where(np.abs(error) > 0.1))

        errs = []
        errs_percent = []
        dists = []
        for i in range(error.shape[0] - 1):
            for j in range(i + 1, error.shape[0]):
                # for i in range(error.shape[0]):
                #     for j in range(error.shape[0]):
                errs.append(error[i, j])
                dists.append(pdist_tls[i, j])
                errs_percent.append(error[i, j] / pdist_tls[i, j])

        errs = np.asarray([err * 100 for err in errs])  # m -> cm

        # 3) remove system errors
        if self.REMOVE_SYSTEM_ERROR:
            errs = self.remove_system_error(errs)

        mu, std = scipy.stats.norm.fit(errs)
        rsme = np.sqrt((errs ** 2).mean())
        print('(%s) RIDE: standard deviation: %f' % (method, std))

        fig = plt.figure(figsize=(13, 9))
        height, bins, patches = plt.hist(errs, bins=40, density=True, stacked=True, alpha=0.6)

        x = np.linspace(mu - 3 * std, mu + 3 * std, 100)
        plt.plot(x, scipy.stats.norm.pdf(x, mu, std), linewidth=5, color='r')

        pt1 = [std, 0]
        pt2 = [std, scipy.stats.norm.pdf(std, mu, std)]
        plt.plot([pt1[0], pt2[0]], [pt1[1], pt2[1]], 'r--', linewidth=5)

        pt1 = [-std, 0]
        pt2 = [-std, scipy.stats.norm.pdf(-std, mu, std)]
        plt.plot([pt1[0], pt2[0]], [pt1[1], pt2[1]], 'r--', linewidth=5)

        plt.xlim([-10, 10])
        plt.ylim([0, 0.38])
        plt.grid()
        plt.title('RIDE: PDF V.S. Error (cm)', fontsize=self.fontsize+5)
        plt.show()



    def targetPositioningError(self, method):
        read_path = os.path.join(self.PRJ_PATH, "MLS_sample_results", method, "target_aligned/")
        pcd_tls = o3d.io.read_point_cloud(read_path + "pcd_tls.pcd")
        pcd_mls = o3d.io.read_point_cloud(read_path + "pcd_mls_icp_aligned.pcd")

        pts_tls = np.asarray(pcd_tls.points)
        pts_mls = np.asarray(pcd_mls.points)
        # plot_targets(pts_tls, pts_mls)
        errs = np.sqrt(np.sum((pts_mls - pts_tls) ** 2, axis=1)) * 100  # m --> cm

        fig = plt.figure(figsize=(13, 9))
        height, bins, patches = plt.hist(errs, bins=12, density=False, stacked=True, alpha=0.6)

        plt.xlim([0, 9])
        plt.ylim([0, 15])

        mu, std = scipy.stats.norm.fit(errs)

        pt1 = [mu, 0]
        pt2 = [mu, 15]
        plt.plot([pt1[0], pt2[0]], [pt1[1], pt2[1]], 'r--', linewidth=5)

        plt.grid()
        plt.title('ATPE: No. V.S. Error (cm)', fontsize=self.fontsize)
        plt.show()

        mu, std = scipy.stats.norm.fit(errs)
        rsme = np.sqrt((errs ** 2).mean())
        print('(%s) ATPE: mean error: %f' % (method, mu))



if __name__ == '__main__':
    BM = Benchmark()
    BM.RIDE()
    BM.ATPE()
