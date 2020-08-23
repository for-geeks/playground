#!/usr/bin/env python
# -*- coding: utf-8 -*-
# How to use iterative closest point
# http://pointclouds.org/documentation/tutorials/iterative_closest_point.php#iterative-closest-point

import pcl
import random
import numpy as np

# from pcl import icp, gicp, icp_nl

def main():
    # cloud_in = pcl.PointCloud()
    cloud_in = pcl.load('/python-pcl/exercises/vehicle_in.pcd')
    print("cloud points : " + str(cloud_in.size))
    
    print('Saved ' + str(cloud_in.size) + ' data points to input:')
    points_out = np.zeros((cloud_in.size, 3), dtype=np.float32)

    print('size:' + str(points_out.size))
    for i in range(0, cloud_in.size):
        points_out[i][0] = cloud_in[i][0] + 0.7
        points_out[i][1] = cloud_in[i][1]
        points_out[i][2] = cloud_in[i][2]

    cloud_out = pcl.PointCloud()
    cloud_out.from_array(points_out)
    pcl.save(cloud_out, 'vehicle_out.pcd')

    print('Transformed ' + str(cloud_out.size) + ' data points:')

    for i in range(0, cloud_out.size):
        print('     ' + str(cloud_out[i][0]) + ' ' + str(cloud_out[i]
                                                         [1]) + ' ' + str(cloud_out[i][2]) + ' data points:')

    # TODO align by icp 
    icp = cloud_in.make_IterativeClosestPoint()
    converged, transf, estimate, fitness = icp.icp(cloud_in, cloud_out，max_iter=10)
    print('has converged:' + str(converged) + ' score: ' + str(fitness))
    print(str(transf))

    transformed_points = transf[0:2, 0:2] * points_out
    transformed_pc.from_array(transformed_points)

    # viewer
    viewer = pcl.pcl_visualization.PCLVisualizering('3D Viewer')
    # cloud_in
    pccolor_in = pcl.pcl_visualization.PointCloudColorHandleringCustom(
        cloud_in, 255, 255, 255)
    # cloud_out
    pccolor_out = pcl.pcl_visualization.PointCloudColorHandleringCustom(
        cloud_out, 255, 0, 0)
    # transformed cloud
    pccolor_transformed = pcl.pcl_visualization.PointCloudColorHandleringCustom(
        transformed_pc, 20, 180, 20)
    
    # Add cloud_in to viewer
    viewer.AddPointCloud_ColorHandler(cloud_in, pccolor_in)
    viewer.AddPointCloud_ColorHandler(cloud_out, pccolor_out)
    

    if converged and not(viewer.WasStopped()):
        # viewer.updatePointCloud(cloud_out, pccolor_out, "cloud_out_v2")
        viewer.AddPointCloud_ColorHandler(transformed_pc, pccolor_transformed, "transformed")
        viewer.SpinOnce()



if __name__ == "__main__":
    main()
    
