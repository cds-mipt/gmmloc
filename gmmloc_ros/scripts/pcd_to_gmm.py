#!/usr/bin/env python

import argparse
import rospy
import pcl
import numpy as np

from sklearn.mixture import GaussianMixture
from collections import defaultdict
from tqdm import tqdm
from gmmloc import GMM_pb2
from google.protobuf.internal.encoder import _VarintBytes


np.warnings.filterwarnings('ignore')


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('--pcd', type=str, required=True)
    parser.add_argument('--gmm', type=str, required=True)
    parser.add_argument('--voxel_size', type=float, required=True)
    return parser


def main(args):
    rospy.init_node('pcd_to_gmm')

    pc = pcl.load(args.pcd)
    X = pc.to_array()

    voxel_grid = defaultdict(list)
    for x, y, z in tqdm(X):
        i, j, k = int(x / args.voxel_size), int(y / args.voxel_size), int(z / args.voxel_size)
        voxel_grid[(i, j, k)].append(([x, y, z]))
    
    for k, v in tqdm(voxel_grid.items()):
        voxel_grid[k] = np.array(v)

    means = {}
    covs = {}
    for ijk, points in tqdm(voxel_grid.items()):
        if len(points) > 1:
            means[ijk] = np.mean(points, axis=0)
            covs[ijk] = np.cov(points, rowvar=False)
    
    # N = 1000
    # gmm = GaussianMixture(N, verbose=2, verbose_interval=1, tol=1e-3)
    # gmm.fit(X)

    with open(args.gmm, 'wb') as f:
        f.write(_VarintBytes(len(means)))
        for ijk in tqdm(means):
            mean = means[ijk]
            cov = covs[ijk]

            gmm = GMM_pb2.ComponentProto()
            gmm.is_degenerated = False
            gmm.is_salient = False
            gmm.mean.extend(mean.tolist())
            gmm.covariance.extend(cov.flatten().tolist())

            f.write(_VarintBytes(gmm.ByteSize()))
            f.write(gmm.SerializeToString())


if __name__ == '__main__':
    parser = build_parser()
    args, _ = parser.parse_known_args()
    main(args)
