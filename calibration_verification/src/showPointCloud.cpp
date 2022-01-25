/*
 * @Author: your name
 * @Date: 2021-09-06 07:57:53
 * @LastEditTime: 2021-09-08 09:51:40
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /calibration_verification/src/showPointCloud.cpp
 */
#include <iostream>
#include <Eigen/Core>
#include <string>
#include <Eigen/Geometry>

#include "pcl/io/ply_io.h"
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_cloud.h>

#include <gflags/gflags.h>

DEFINE_string(file_path,"../data/PointCloud_6.ply","path");

void readPLY(std::string &plyfilename)
{
    pcl::PointCloud<pcl::PointXYZ>  plyCloud;
    if (pcl::io::loadPLYFile(plyfilename, plyCloud )<0 )
    {
        std::cout << "error";
    }
    pcl::visualization::PCLVisualizer::Ptr viewer2 (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer2->setBackgroundColor (0, 0, 0);
    viewer2->addPointCloud<pcl::PointXYZ> (plyCloud.makeShared(), "sample cloud");
    viewer2->initCameraParameters ();
    viewer2->addCoordinateSystem(0.2);
    while (!viewer2->wasStopped())
     {
        viewer2->spinOnce(10);
    }
}

int main( int argc , char** argv)
{
    google::ParseCommandLineFlags(&argc,&argv,true);
    std::string ply_file_path = FLAGS_file_path;
    readPLY(ply_file_path);
}