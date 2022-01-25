/*
 * @Author: your name
 * @Date: 2021-09-13 15:36:27
 * @LastEditTime: 2021-09-14 14:08:11
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /calibration_verification/src/showGround.cpp
 */
/*
 * @Author: jiangxinyu
 * @Date: 2021-08-25 10:38:17
 * @LastEditTime: 2021-09-06 14:50:08
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

#include <pcl/sample_consensus/model_types.h>   //分割模型的头文件
#include <pcl/sample_consensus/method_types.h>   //采样一致性的方法
#include <pcl/segmentation/sac_segmentation.h>  //ransac分割法
#include <pcl/common/impl/centroid.hpp>
#include <pcl/common/common_headers.h>

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
ros::Publisher pub2;
// 初始化外参矩阵RotationMat，及绕每个轴的旋转矩阵;
Eigen::Matrix3d RotationMat = Eigen::Matrix3d::Identity();
// 位移矩阵
Eigen::Vector3d transVec(0,0,0);
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void  calNormalVectorByRansac( pcl::PointCloud<pcl::PointXYZRGB> &flatCloud , Eigen::Vector4d &normalVector , pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane )
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr flat_;
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());  //申明存储模型的内点的索引
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());  //申明模型的参数
    seg.setOptimizeCoefficients (true);
    // 以下都是强制性的需要设置的
    seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);   //平面模型
    seg.setMethodType (pcl::SAC_RRANSAC);    //分割平面模型所使用的分割方法
    seg.setDistanceThreshold (0.005);        //设置最小的阀值距离
    seg.setMaxIterations(10000);
    float angle = 90;
    float EpsAngle= pcl::deg2rad(angle);   // 角度转弧度
    Eigen::Vector3f Axis(0.0, 0.0, 1.0); 
    seg.setAxis(Axis);    // 指定的轴
    seg.setEpsAngle(EpsAngle);             // 夹角阈值(弧度制)
    seg.setInputCloud (flatCloud.makeShared());   //设置输入的点云
    seg.segment (*inliers, *coefficients);
    //  取出平面的内点形成点云
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud(flatCloud.makeShared());
    extract.setIndices(inliers);
    //除去平面之外的数据
    extract.setNegative(false);
    extract.filter(*cloud_plane);
    // 打印
    std::cout << "RANSAC 平面内点数量为 :  "<< cloud_plane ->size() << "\n"
                    <<"平面模型系数为 : \n" 
                    << "A = " << coefficients->values[0] << "\n" 
                    << "B = " << coefficients->values[1] << "\n" 
                    << "C = " << coefficients->values[2] << "\n" 
                    << "D ="  << coefficients->values[3] << "\n";
    normalVector << coefficients->values[0] ,coefficients->values[1],coefficients->values[2],coefficients->values[3];
}
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
    if (point.y >= 0 && point.y < 0.005)
    {
        point.r = 0;
        point.g = 0;
        point.b = 0;
        return;
    }
    if (point.y >=  0.005 && point.y < 0.01)
    {
        point.r = 125;
        point.g = 125;
        point.b = 125;
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
    sensor_msgs::PointCloud2 cloud_ground;
    sensor_msgs::PointCloud2 fitted_ground;
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
        if ( curPoint.z < 0.3 && curPoint.z > 0.15 && curPoint.x < 0.3 && curPoint.x > -0.3)
        {
            colored_pcl_ptr->points.push_back(curPoint);
        }
    }//for
    colored_pcl_ptr->width = 1;
    colored_pcl_ptr->height = colored_pcl_ptr->points.size();
    // 
    std::cout << "colored_pcl_ptr->points size  = " << colored_pcl_ptr->points.size() << "\n";

    /*
    *   Ransca get plane param  , publish inliers
    */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr planeCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    Eigen::Vector4d planeParam;
    calNormalVectorByRansac(*colored_pcl_ptr,planeParam,planeCloud);
    pcl::toROSMsg( *planeCloud,  cloud_ground);  //将点云转化为消息才能发布
    cloud_ground.header.frame_id = "base_link";
    pub1.publish(cloud_ground);

    /*
    *   build new pointcloud with plane param  
    */
    pcl::PointCloud<pcl::PointXYZRGB> fittedPlane;
    const double step = 0.003;
    const double x_begin = -0.3;
    const double x_end = 0.3;
    const double z_begin = -0.3;
    const double z_end = 0.3;

    auto cal_y_value = [&](double x ,double z)
    {
        auto paramA = planeParam[0];
        auto paramB = planeParam[1];
        auto paramC = planeParam[2];
        auto paramD = planeParam[3];
        return -(paramA*x + paramC*z +paramD)/paramB;
    };

    for (auto x_value = x_begin; x_value < x_end; x_value+=step)
    {
        for (auto z_value = z_begin; z_value < z_end; z_value+=step)
        {
            pcl::PointXYZRGB curPoint;
            curPoint.x = x_value;
            curPoint.z = z_value;
            curPoint.y = cal_y_value(curPoint.x,curPoint.z);
            // curPoint.r = 0;
            // curPoint.g =0;
            // curPoint.b = 255;
            setColor(curPoint);
            fittedPlane.points.push_back(curPoint);
        }
    }
    fittedPlane.width = 1;
    fittedPlane.height = fittedPlane.points.size();
    pcl::toROSMsg( fittedPlane,  fitted_ground);  //将点云转化为消息才能发布
    fitted_ground.header.frame_id = "base_link";
    pub2.publish(fitted_ground);
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

    ros::init(argc, argv, "showGround_node");
    ros::NodeHandle n;
    ros::Subscriber calNormalSuber = n.subscribe("pointcloud_3d", 50, cloud_call_back);
    pub1 = n.advertise<sensor_msgs::PointCloud2>("cloud_ground", 50);
    pub2 = n.advertise<sensor_msgs::PointCloud2>("fitted_ground", 50);
    ros::spin();
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////