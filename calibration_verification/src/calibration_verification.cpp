/*
 * @Author: your name
 * @Date: 2021-08-19 16:04:41
 * @LastEditTime: 2021-08-31 18:56:43
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /calibration_verification/src/main.cpp
 */

// ros
#include"ros/ros.h"
#include "ros/common.h"
#include "sensor_msgs/Image.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
// glags
#include <gflags/gflags.h>
//  
#include "planeDetection/planeDetection.h"

// 定义gflags参数
DEFINE_double(Rx,0,"The rotation angle of the camera coordinate system around the x axis in the robot coordinate system");
DEFINE_double(Ry,0,"The rotation angle of the camera coordinate system around the y axis in the robot coordinate system");
DEFINE_double(Rz,0,"The rotation angle of the camera coordinate system around the z axis in the robot coordinate system");
DEFINE_double(Tx,0,"The X translation of the camera coordinate system in the robot coordinate system");
DEFINE_double(Ty,0,"The Y translation of the camera coordinate system in the robot coordinate system");
DEFINE_double(Tz,0,"The Z translation of the camera coordinate system in the robot coordinate system");

// 初始化外参矩阵RotationMat，及绕每个轴的旋转矩阵;
Eigen::Matrix3d RotationMat = Eigen::Matrix3d::Identity();
// 位移矩阵
Eigen::Vector3d transVec(0,0,0);
// 内参矩阵
Eigen::Matrix3d CameraKMat = Eigen::Matrix3d::Identity();

const double cx = 111.430519;
const double cy = 57.936710;
const double fx = 80.108238;
const double fy = 80.108238;
const int width =224;
const int height = 114;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void callback(const sensor_msgs::ImageConstPtr &ir_image,const sensor_msgs::ImageConstPtr &depth_image)
{
    std::cout <<"<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<"<<"\n";
    std::cout << "IR & Depth Time = "<< ir_image->header.stamp << "," << depth_image->header.stamp<<"\n";

    //  transform image to cv::Mat
    cv::Mat ir_src_in;
    cv::Mat depth_src_in;
    // copy depth_image
    {
        uint16_t* depthData [width][height];
        memcpy(depthData,depth_image->data.data(),depth_image->data.size()*sizeof(uint16_t));
        Mat tmp (Size(width, height), CV_16UC1, depthData);
        depth_src_in = tmp.clone();
    }
    // copy ir_image
    {
        uint8_t* irData [width][height];
        memcpy(irData,ir_image->data.data(),ir_image->data.size()*sizeof(uint16_t));
        Mat src (Size(width, height), CV_16UC1, irData);
        convertScaleAbs(src,src);
        ir_src_in = src.clone();
    }   // 标定检测
    calibration_verification_fun(ir_src_in,depth_src_in);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc , char** argv  )
{
    std::cout << "Main :  \n";
    // 读取参数
    google::ParseCommandLineFlags(&argc,&argv,true);
    const double Rx = FLAGS_Rx;
    const double Ry = FLAGS_Ry;
    const double Rz = FLAGS_Rz;
    setCameraKMat(CameraKMat, fx, fy, cx, cy); // 设置内参
    setRotationMat(RotationMat,Rx,Ry,Rz);  // 设置外参
    transVec << FLAGS_Tx , FLAGS_Ty , FLAGS_Tz;
    std::cout << "外参：\n" << "Rx = " << Rx << "," << "Ry = " << Ry << "," << "Rz = " << Rz << "\n" 
                    << "Tx = " << transVec[0] << "," << "Ty = " << transVec[1] << "," << "Tz = " << transVec[2] << "\n";
    // 
    ros::init(argc , argv, "calibration_verification_node");
    ros::NodeHandle nh;
    int camera_queue_size = 1;
    std::string ir_topic = "ir_image";
    std::string depth_topic = "depth_image";
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> ApproxSync;
    message_filters::Subscriber<sensor_msgs::Image> m_ir_sub(nh, "ir_image", camera_queue_size);
    message_filters::Subscriber<sensor_msgs::Image> m_depth_sub(nh, "depth_image", camera_queue_size);
    message_filters::Synchronizer<ApproxSync> sync(ApproxSync(10), m_ir_sub, m_depth_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));
    ros::Rate r(5);
     
    while (nh.ok())
    {
        ros::spinOnce();
        r.sleep();
    }
    gflags::ShutDownCommandLineFlags();
}