#!/usr/bin/env python
# -*- coding: utf-8 -*-
# How to use iterative closest point
# http://pointclouds.org/documentation/tutorials/iterative_closest_point.php#iterative-closest-point

import pcl
import random
import numpy as np

# from pcl import icp, gicp, icp_nl


def main():
    cloud_in = pcl.PointCloud()
    cloud_out = pcl.PointCloud()

    # Fill in the CloudIn data
    points_in = np.zeros((5, 3), dtype=np.float32)
    RAND_MAX = 1024.0
    for i in range(0, 5):
        points_in[i][0] = 1024 * random.random() / RAND_MAX
        points_in[i][1] = 1024 * random.random() / RAND_MAX
        points_in[i][2] = 1024 * random.random() / RAND_MAX

    cloud_in.from_array(points_in)

    print('Saved ' + str(cloud_in.size) + ' data points to input:')
    points_out = np.zeros((5, 3), dtype=np.float32)

    print('size:' + str(points_out.size))
    for i in range(0, cloud_in.size):
        points_out[i][0] = points_in[i][0] + 0.7
        points_out[i][1] = points_in[i][1]
        points_out[i][2] = points_in[i][2]

    cloud_out.from_array(points_out)

    print('Transformed ' + str(cloud_in.size) + ' data points:')

    for i in range(0, cloud_out.size):
        print('     ' + str(cloud_out[i][0]) + ' ' + str(cloud_out[i]
                                                         [1]) + ' ' + str(cloud_out[i][2]) + ' data points:')

    icp = cloud_in.make_IterativeClosestPoint()
    # Final = icp.align()
    converged, transf, estimate, fitness = icp.icp(cloud_in, cloud_out)

    print('has converged:' + str(converged) + ' score: ' + str(fitness))
    print(str(transf))


if __name__ == "__main__":
    main()
    