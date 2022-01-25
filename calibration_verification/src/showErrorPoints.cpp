/*
 * @Author: your name
 * @Date: 2021-08-19 16:04:41
 * @LastEditTime: 2021-09-06 09:44:19
 * @LastEditors: Please set LastEditors
 * @Description: 显示深度值较小的点
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
#include <iostream>
#include "std_msgs/String.h"
#include "geometry_msgs/TransformStamped.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud.h"
#include <opencv2/opencv.hpp>
#include "image_transport/image_transport.h"
#include "opencv2/highgui/highgui.hpp"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace cv;

#ifndef DEG2RAD
#define DEG2RAD(x) ((x)*0.017453293)
#endif

#ifndef RAD2DEG
#define RAD2DEG(x) ((x)*57.29578)
#endif

// 定义gflags参数
DEFINE_double(Rx,0,"The rotation angle of the camera coordinate system around the x axis in the robot coordinate system");
DEFINE_double(Ry,0,"The rotation angle of the camera coordinate system around the y axis in the robot coordinate system");
DEFINE_double(Rz,0,"The rotation angle of the camera coordinate system around the z axis in the robot coordinate system");
DEFINE_double(Tx,0,"The X translation of the camera coordinate system in the robot coordinate system");
DEFINE_double(Ty,0,"The Y translation of the camera coordinate system in the robot coordinate system");
DEFINE_double(Tz,0,"The Z translation of the camera coordinate system in the robot coordinate system");
// 定义gflags参数
DEFINE_double(distance,0,"The max diatance");

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

const int width = 224;
const int height = 114;
double distanceThr;

typedef union USHORT_UNION
{
    ushort num;
    char c [2];
} ushort_union;

ushort Little(char c0 ,char c1)
{
    ushort_union tmp;
    tmp.c[0] = c0;
    tmp.c[1] = c1;
    return tmp.num;
}

ros::Publisher pub1;
void callback(const sensor_msgs::ImageConstPtr &ir_image, const sensor_msgs::ImageConstPtr &depth_image);
void getNoiseImageMat(const cv::Mat &ir_ , const cv::Mat &depth_ , Eigen::Matrix3d &CameraKMat_ , cv::Mat &output_image);
void getGroundNoiseImageMat(const cv::Mat &ir_ , const cv::Mat &depth_ , Eigen::Matrix3d &CameraKMat_ , 
                                                       const Eigen::Matrix3d  &RotationMat_ , const Eigen::Vector3d &transVec_ ,cv::Mat &output_image);
void setCameraKMat(Eigen::Matrix3d &CameraKMat_ ,double fx_,double fy_ ,double cx_,double cy_);
void setRotationMat(Eigen::Matrix3d &RotationMat_ ,double angleX_,double angleY_,double angleZ_);
void transformPoints(const Eigen::Vector3d &pointVec,  const Eigen::Matrix3d &rotationMatrix , const Eigen::Vector3d &transVector , Eigen::Vector3d &outputVec);
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void callback(const sensor_msgs::ImageConstPtr &ir_image, const sensor_msgs::ImageConstPtr &depth_image)
{
    std::cout <<"<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<"<<"\n";
    std::cout << "PointCloud & Depth Time = "<< ir_image->header.stamp << "," << depth_image->header.stamp<<"\n";
    std::cout << "distanceThr = " << distanceThr << "\n";
    cv::Mat ir_src_in;
    cv::Mat depth_src_in;
    // copy depth_image
    {
        uint16_t* depthData [width][height];
        memcpy(depthData,depth_image->data.data(),depth_image->data.size()*sizeof(uint16_t));
        cv::Mat tmp (Size(width, height), CV_16UC1, depthData);
        depth_src_in = tmp.clone();
    }
    // copy ir_image
    {
        uint8_t* irData [width][height];
        memcpy(irData,ir_image->data.data(),ir_image->data.size()*sizeof(uint16_t));
        cv::Mat src (Size(width, height), CV_16UC1, irData);
        cv::convertScaleAbs(src,src);
        ir_src_in = src.clone();
    }
    cv::Mat outputImage;

    // getNoiseImageMat(ir_src_in,depth_src_in,CameraKMat,outputImage);
    getGroundNoiseImageMat(ir_src_in,depth_src_in,CameraKMat,RotationMat,transVec,outputImage);

    // cvNamedWindow("outputImage", 0);
	// imshow("outputImage", outputImage);
	// cvWaitKey(0);
    // cv::destroyWindow("outputImage");

    // 显示
    outputImage.convertTo(outputImage,CV_16U);
    sensor_msgs::Image showNoiseImage;
    showNoiseImage.encoding = "16UC1";
    showNoiseImage.width = ir_image->width;
    showNoiseImage.height = ir_image->height;
    showNoiseImage.header.frame_id = ir_image->header.frame_id;
    showNoiseImage.header.stamp = ir_image->header.stamp;
    showNoiseImage.data.resize(ir_image->data.size());
    memcpy(showNoiseImage.data.data(), outputImage.data, ir_image->data.size()*sizeof(uint8_t));
    std::cout << ir_image->data.size() << "\n";
    std::cout << showNoiseImage.data.size() << "\n";
    pub1.publish(showNoiseImage);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void transformPoints(const Eigen::Vector3d &pointVec,  const Eigen::Matrix3d &rotationMatrix , const Eigen::Vector3d &transVector , Eigen::Vector3d &outputVec)
{
    outputVec = rotationMatrix*pointVec + transVector;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void getNoiseImageMat(const cv::Mat &ir_ , const cv::Mat &depth_ , Eigen::Matrix3d &CameraKMat_ , cv::Mat &output_image)
{
    output_image = ir_.clone();
    std::vector<Eigen::Vector3d> pixelCoordinates;
    std::vector<Eigen::Vector3d> cameraCoordinates;
    // 从图像中获取像素坐标及深度指
    for (int i = 0; i < ir_.rows;i++)
    {
        for(int j =0 ;j < ir_.cols;j++)
        {
            double depth_value = depth_.at<uint16_t>(i,j)/1000.0;
            
            Eigen::Vector3d uv(i,j,depth_value);
            pixelCoordinates.push_back(uv);
            Eigen::Vector3d pointInCamera;
            pointInCamera << uv[2]*(uv[1] - CameraKMat_(0,2))/CameraKMat_(0,0),
                                            uv[2]*(uv[1] - CameraKMat_(0,2))/CameraKMat_(0,0),
                                            uv[2];
            if (  pointInCamera[2] != 0 && pointInCamera[2] < distanceThr && pointInCamera[1] < 0 )
            {
                output_image.at<uint8_t>(i,j) = 0;
            }else
            {
                output_image.at<uint8_t>(i,j) = 255;
            }
        }
    } // for
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void getGroundNoiseImageMat(const cv::Mat &ir_ , const cv::Mat &depth_ , Eigen::Matrix3d &CameraKMat_ , 
                                                       const Eigen::Matrix3d  &RotationMat_ , const Eigen::Vector3d &transVec_ ,cv::Mat &output_image)
{
    output_image = ir_.clone();
    std::vector<Eigen::Vector3d> pixelCoordinates;
    std::vector<Eigen::Vector3d> cameraCoordinates;
    // 从图像中获取像素坐标及深度指
    for (int i = 0; i < ir_.rows;i++)
    {
        for(int j =0 ;j < ir_.cols;j++)
        {
            double depth_value = depth_.at<uint16_t>(i,j)/1000.0;
            Eigen::Vector3d uv(i,j,depth_value);
            pixelCoordinates.push_back(uv);
            Eigen::Vector3d pointInCamera;
            pointInCamera << uv[2]*(uv[1] - CameraKMat_(0,2))/CameraKMat_(0,0),
                                            uv[2]*(uv[1] - CameraKMat_(0,2))/CameraKMat_(0,0),
                                            uv[2];
            Eigen::Vector3d pointInWorld;
            transformPoints(pointInCamera,RotationMat_,transVec_,pointInWorld);

            if (  pointInWorld[2] > 0.25  && pointInWorld[2] < 1 && pointInWorld[1] <= -0.01 && pointInWorld[1] > -0.03 )
            {
                std::cout << "Point = " << pointInWorld[0] << "," << pointInWorld[1] << "," << pointInWorld[2] << "\n";
                output_image.at<uint8_t>(i,j) = 0;
            }else
            {
                output_image.at<uint8_t>(i,j) = 255;
            }
        }
    } // for
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setCameraKMat(Eigen::Matrix3d &CameraKMat_ ,double fx_,double fy_ ,double cx_,double cy_)
{
    Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
    K << fx_ , 0 , cx_,
            0 , fy_ ,cy_,
            0,0,1;
    CameraKMat_ = K;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc , char** argv  )
{
    std::cout << "Main :  \n";
    // 读取参数
    google::ParseCommandLineFlags(&argc,&argv,true);
    distanceThr  = FLAGS_distance;
    const double Rx = FLAGS_Rx;
    const double Ry = FLAGS_Ry;
    const double Rz = FLAGS_Rz;

    setCameraKMat(CameraKMat, fx, fy, cx, cy); // 设置内参
    setRotationMat(RotationMat,Rx,Ry,Rz);  // 设置外参
    transVec << FLAGS_Tx , FLAGS_Ty , FLAGS_Tz;

    ros::init(argc , argv, "showErrorPoints_node");
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
    pub1 = nh.advertise<sensor_msgs::Image>("depth_show_error", 50);
     
    while (nh.ok())
    {
        ros::spinOnce();
        r.sleep();
    }
    gflags::ShutDownCommandLineFlags();
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////