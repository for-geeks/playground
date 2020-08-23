#!/usr/bin/env python
# -*- coding: utf-8 -*-

import pcl
import pcl.pcl_visualization
import random
import numpy as np

def main():
    # cloud_in = pcl.PointCloud()
    cloud_in = pcl.load('/python-pcl/exercises/vehicle_in.pcd')
    print("cloud points : " + str(cloud_in.size))
    
    print('Saved ' + str(cloud_in.size) + ' data points to input')
    points_out = np.zeros((cloud_in.width, 3), dtype=np.float32)

    print('size:' + str(points_out.size))
    for i in range(0, cloud_in.size):
        points_out[i][0] = cloud_in[i][0] + 1.7
        points_out[i][1] = cloud_in[i][1]
        points_out[i][2] = cloud_in[i][2]

    cloud_out = pcl.PointCloud()
    cloud_out.from_array(points_out)
    #pcl.save(cloud_out, 'vehicle_out.pcd')

    print('Transformed ' + str(cloud_out.size) + ' data points')

    # Debug print all points 
    #for i in range(0, cloud_out.size):
    #    print('     ' + str(cloud_out[i][0]) + ' ' + str(cloud_out[i]
    #                                                     [1]) + ' ' + str(cloud_out[i][2]) + ' data points:')

    # TODO align by icp 
    icp = cloud_in.make_IterativeClosestPoint()
    converged, transf, estimate, fitness = icp.icp(cloud_in, cloud_out, max_iter=50)
    print('has converged:' + str(converged) + ' score: ' + str(fitness))
    print(str(transf))

    transformed_points = np.dot(cloud_in, transf[0:3, 0:3])
    transformed_pc = pcl.PointCloud()
    transformed_pc.from_array(transformed_points)
    pcl.save(cloud_out, 'transformed_vehicle_out.pcd')
    # viewer
    viewer = pcl.pcl_visualization.PCLVisualizering('3D Viewer')
    # cloud_in in white color
    pccolor_in = pcl.pcl_visualization.PointCloudColorHandleringCustom(
        cloud_in, 255, 255, 255)
    # cloud_out in red color
    pccolor_out = pcl.pcl_visualization.PointCloudColorHandleringCustom(
        cloud_out, 180, 20, 20)
    # transformed cloud in green color
    pccolor_transformed = pcl.pcl_visualization.PointCloudColorHandleringCustom(
        transformed_pc, 20, 180, 20)
    # Add cloud_in to viewer
    viewer.AddPointCloud_ColorHandler(cloud_in, pccolor_in, "cloud_in")
    viewer.AddPointCloud_ColorHandler(cloud_out, pccolor_out, "cloud_out")
    viewer.AddPointCloud_ColorHandler(transformed_pc, pccolor_transformed, "transformed_cloud")

    while not(viewer.WasStopped()):
        #viewer.updatePointCloud(cloud_out, pccolor_out, "cloud_out_v2")
        viewer.SpinOnce()


if __name__ == "__main__":
    main()
    