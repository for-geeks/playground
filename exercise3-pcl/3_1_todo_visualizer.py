#!/usr/bin/env python
# run python3 visualizer.py
import pcl
import pcl.pcl_visualization
import numpy as np

def main():

    cloud = pcl.load('/python-pcl/apollo_scape_pcd/333.pcd')
    print("cloud points : " + str(cloud.size))

    viewer = pcl.pcl_visualization.PCLVisualizering('3D Viewer')
    pccolor = pcl.pcl_visualization.PointCloudColorHandleringCustom(
        cloud, 255, 255, 255)

    # viewer 
    viewer.AddPointCloud_ColorHandler(cloud, pccolor)

    v = True
    while v:
        v = not(viewer.WasStopped())
        viewer.SpinOnce()
        # sleep(0.01)


if __name__ == "__main__":
    # import cProfile
    # cProfile.run('main()', sort='time')
    main()
