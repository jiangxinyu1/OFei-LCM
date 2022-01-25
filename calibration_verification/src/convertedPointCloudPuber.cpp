/*
 * @Author: jiangxinyu
 * @Date: 2021-08-25 10:38:17
 * @LastEditTime: 2021-09-14 18:27:15
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /calibration_verification/src/convertedPointCloudPuber.cpp
 */

#include <iostream>
#include "ros/ros.h"

#include "ros/ros.h"
#include <iostream>
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Point.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud2_iterator.h"
#include "sensor_msgs/Image.h"
// pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
// Eigen
#include <Eigen/Dense>

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <gflags/gflags.h>

// 定义gflags参数
DEFINE_double(Rx,0,"The rotation angle of the camera coordinate system around the x axis in the robot coordinate system");
DEFINE_double(Ry,0,"The rotation angle of the camera coordinate system around the y axis in the robot coordinate system");
DEFINE_double(Rz,0,"The rotation angle of the camera coordinate system around the z axis in the robot coordinate system");
DEFINE_double(Tx,0,"The X translation of the camera coordinate system in the robot coordinate system");
DEFINE_double(Ty,0,"The Y translation of the camera coordinate system in the robot coordinate system");
DEFINE_double(Tz,0,"The Z translation of the camera coordinate system in the robot coordinate system");

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool blMiddle( double target ,double min_ ,double max_ );
void cloud_call_back(const sensor_msgs::PointCloudConstPtr& input);
void setColor(pcl::PointXYZRGB &point);
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
ros::Publisher pub1;
// 初始化外参矩阵RotationMat，及绕每个轴的旋转矩阵;
Eigen::Matrix3d RotationMat = Eigen::Matrix3d::Identity();
// 位移矩阵
Eigen::Vector3d transVec(0,0,0);
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Set the Rotation Mat object
 *               
 * @param RotationMat_  绕固定轴XYZ欧拉角旋转矩阵
 * @param angleX_ 单位度
 * @param angleY_ 
 * @param angleZ_ 
 */
void setRotationMat(Eigen::Matrix3d &RotationMat_ ,double angleX_,double angleY_,double angleZ_)
{
    Eigen::Matrix3d Rotation_Y = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d Rotation_Z = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d Rotation_X = Eigen::Matrix3d::Identity();
    // X
    double cosAngleX = std::cos(DEG2RAD(angleX_));
    double sinAngleX = std::sin(DEG2RAD(angleX_));
    Rotation_X << 1,0,0,
                              0,cosAngleX,-sinAngleX,
                              0,sinAngleX,cosAngleX;
    // Y
    double cosAngleY = std::cos(DEG2RAD(angleY_));
    double sinAngleY = std::sin(DEG2RAD(angleY_));
    Rotation_Y <<  cosAngleY, 0, sinAngleY,
                               0,1,0,
                               -sinAngleY,0,cosAngleY;
    // Z
    double cosAngleZ = std::cos(DEG2RAD(angleZ_));
    double sinAngleZ = std::sin(DEG2RAD(angleZ_));
    Rotation_Z << cosAngleZ,-sinAngleZ,0,
                               sinAngleZ,cosAngleZ,0,
                               0,0,1;

    RotationMat_ = Rotation_Z * Rotation_Y * Rotation_X;   
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 通过外参矩阵转换 point 
void transformPoints(pcl::PointXYZRGB &point,  const Eigen::Matrix3d &rotationMatrix , const Eigen::Vector3d &transVector)
{
    Eigen::Vector3d pointVec(point.x,point.y,point.z);
    Eigen::Vector3d outPointVec ;
    outPointVec = rotationMatrix*pointVec + transVector;
    point.x = outPointVec[0];
    point.y = outPointVec[1];
    point.z = outPointVec[2];
}
void transformPoints(Eigen::Vector3d &pointVec,  const Eigen::Matrix3d &rotationMatrix , const Eigen::Vector3d &transVector)
{
    Eigen::Vector3d outPointVec ;
    outPointVec = rotationMatrix*pointVec + transVector;
    pointVec[0]= outPointVec[0];
    pointVec[1]= outPointVec[1];
    pointVec[2]= outPointVec[2];
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setColor(pcl::PointXYZRGB &point)
{
    if (blMiddle(point.y,-0.005,0) || point.y == 0 ) //  [-0.005,0) 
    {
        point.r = 0;
        point.g = 0;
        point.b = 255;
        return ;
    }
    if (blMiddle(point.y,-0.01,-0.005)) // [-0.01,-0.005)
    {
        point.r = 0;
        point.g = 255;
        point.b = 0;
        return ;
    }
    if (blMiddle(point.y,-0.015,-0.01)) // [-0.015,-0.01)
    {
        point.r = 255;
        point.g = 0;
        point.b = 0;
        return ;
    }
    if (blMiddle(point.y,-0.02,-0.015)) // [-0.02,-0.015)
    {
        point.r = 0;
        point.g = 255;
        point.b = 255;
        return ;
    }
    if ( blMiddle(point.y,-0.03,-0.02) ) // 
    {
        point.r = 255;
        point.g = 255;
        point.b = 0;
        return;
    }
    if ( point.y <  -0.03) // 
    {
        point.r = 255;
        point.g = 0;
        point.b = 255;
        return;
    }
    if (point.y >= 0 && point.y < 0.005) // heise
    {
        point.r = 0;
        point.g = 0;
        point.b = 0;
        return;
    }
    if (point.y >=  0.005 && point.y < 0.01) // menglan
    {
        point.r = 3;
        point.g = 168;
        point.b = 158;
        return;
    }
    if (point.y >=  0.01 && point.y < 0.015)    // tu
    { 
        point.r = 199;
        point.g = 97;
        point.b = 20;
        return;
    }
    if (point.y >=  0.015 && point.y < 0.02) // 
    {
        point.r = 128;
        point.g = 42;
        point.b = 42;
        return;
    }
    point.r = 255;
    point.g = 255;
    point.b = 255;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief  min <= target < max 
 * 
 * @param target 
 * @param min_ 
 * @param max_ 
 * @return true 
 * @return false 
 */
bool blMiddle( double target ,double min_ ,double max_ )
{
    if (target < max_ && target >= min_)
    {
        return true;
    }
    return false;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void cloud_call_back(const sensor_msgs::PointCloudConstPtr& input)
{
    std::cout << "Start to  convert pointcloud ... \n"; 
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr  colored_pcl_ptr (new pcl::PointCloud<pcl::PointXYZRGB>); 
    sensor_msgs::PointCloud2 cloud_trans;
    int pointNumber = input->points.size();
    for (auto index = 0 ; index < pointNumber; index++)
    {
        double tmp_x =  (double)input->points[index].x;
        double tmp_y =  (double)input->points[index].y;
        double tmp_z =  (double)input->points[index].z;
        Eigen::Vector3d pointRaw(tmp_x,tmp_y,tmp_z);
        transformPoints(pointRaw,RotationMat,transVec);
        pcl::PointXYZRGB curPoint;
        curPoint.x = pointRaw[0];
        curPoint.y = pointRaw[1];
        curPoint.z = pointRaw[2];
        setColor(curPoint);
        colored_pcl_ptr->points.push_back(curPoint);
    }
    colored_pcl_ptr->width = 1;
    colored_pcl_ptr->height = input->points.size();
    pcl::toROSMsg( *colored_pcl_ptr,  cloud_trans);  //将点云转化为消息才能发布
    cloud_trans.header.frame_id = "base_link";
    // 发布外参转换后的点云
    pub1.publish(cloud_trans);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
    google::ParseCommandLineFlags(&argc,&argv,true);
    const double Rx = FLAGS_Rx;
    const double Ry = FLAGS_Ry;
    const double Rz = FLAGS_Rz;
    setRotationMat(RotationMat,Rx,Ry,Rz);  // 设置外参
    transVec << FLAGS_Tx , FLAGS_Ty , FLAGS_Tz;
    std::cout << "外参：\n" << "Rx = " << Rx << "," << "Ry = " << Ry << "," << "Rz = " << Rz << "\n" 
                    << "Tx = " << transVec[0] << "," << "Ty = " << transVec[1] << "," << "Tz = " << transVec[2] << "\n";

    ros::init(argc, argv, "convertedPointCloudPuber_node");
    ros::NodeHandle n;
    ros::Subscriber calNormalSuber = n.subscribe("pointcloud_3d", 50, cloud_call_back);
    pub1 = n.advertise<sensor_msgs::PointCloud2>("cloud_trans", 50);
    ros::spin();
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////