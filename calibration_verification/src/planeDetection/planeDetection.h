/*
 * @Author: your name
 * @Date: 2021-08-19 16:03:27
 * @LastEditTime: 2021-08-23 15:08:18
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /calibration_verification/src/planeDetection.h
 */
#ifndef _PLANE_DETECTION_H_
#define _PLANE_DETECTION_H_

#include <iostream>
#include  <cstdio>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "pcl/io/pcd_io.h"
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/model_types.h>   //分割模型的头文件
#include <pcl/sample_consensus/method_types.h>   //采样一致性的方法
#include <pcl/segmentation/sac_segmentation.h>  //ransac分割法
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/eigen.h>
#include <pcl/common/impl/centroid.hpp>
#include <pcl/common/common_headers.h>
#include"ros/ros.h"
#include "ros/common.h"

#ifndef RAD2DEG
#define RAD2DEG(x) ((x)*180./M_PI)
#endif // !RAD2DEG


using namespace cv;
using namespace std;


// 初始化外参矩阵RotationMat，及绕每个轴的旋转矩阵;
extern Eigen::Matrix3d RotationMat;
// 位移矩阵
extern Eigen::Vector3d transVec;
// 内参矩阵
extern Eigen::Matrix3d CameraKMat;

extern const double cx ;
extern const double cy ;
extern const double fx ;
extern const double fy ;
extern const int width;
extern const int height;

void mySobel(Mat &src,Mat &abs_gradient_x,Mat &abs_gradient_y, Mat &dst3, Mat &direction_);
void myCanny(Mat &src );
void findLine(Mat &outputSoble,Mat &directionSobel , Mat &outputCanny , Mat &outputLine);
void findFlat(Mat &outputLine ,Mat &src , Mat &depth_src ,Mat &outputFlat , const int &grayThreshold, const int &depthThreshold);
cv::Mat regionGrowFast(const cv::Mat &src,  const cv::Mat &depth_src, std::vector<cv::Point2i> &seeds,const int &grayThreshold, const int &depthThreshold );
void toFlatPointCloud(const Mat &flat_ , const Mat &depth_ , pcl::PointCloud<pcl::PointXYZRGB> &flatCloud , 
                                        Eigen::Matrix3d &RotationMat_ ,Eigen::Matrix3d &CameraKMat_ ,Eigen::Vector3d &transVec_);
void binaryImage2PointCloud(const Mat &binaryImage, const Mat &depth_ , pcl::PointCloud<pcl::PointXYZRGB> &cloud , 
                                        Eigen::Matrix3d &RotationMat_ ,Eigen::Matrix3d &CameraKMat_ ,Eigen::Vector3d &transVec_);                                        
void toPointCloud(const Mat &ir_ , const Mat &depth_ , pcl::PointCloud<pcl::PointXYZRGB> &cloud , 
                                        Eigen::Matrix3d &RotationMat_ ,Eigen::Matrix3d &CameraKMat_ ,Eigen::Vector3d &transVec_);
void setRotationMat(Eigen::Matrix3d &RotationMat_ ,double angleX_,double angleY_,double angleZ_);
                                        
void setCameraKMat(Eigen::Matrix3d &CameraKMat_ ,double fx_,double fy_ ,double cx_,double cy_);
void transformPoints(pcl::PointXYZRGB &point,  const Eigen::Matrix3d &rotationMatrix , const Eigen::Vector3d &transVector);
void transformPoints(const Eigen::Vector3d &pointVec,  const Eigen::Matrix3d &rotationMatrix , const Eigen::Vector3d &transVector , Eigen::Vector3d &outputVec);
void readPLY(std::string &plyfilename);
int readFile(std::string &filename , unsigned char* data , const int width , const int height);
int readFile(std::string &filename , uint16_t* data , const int width , const int height);
void findOutlier(Mat &ir_ , Mat &depth_ );
void  calNormalVectorByRansac( pcl::PointCloud<pcl::PointXYZRGB> &flatCloud , Eigen::Vector3d &normalVector , pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane );
void calNormalVecByPCA(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
void calNormalVecByPCA(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud , Eigen::Vector3d &mainDirectoin , Eigen::Vector3d &normalDirection );
void calLineVectorByRansac(pcl::PointCloud<pcl::PointXYZRGB> &lineCloud , Eigen::Vector3d &directionVector , pcl::PointCloud<pcl::PointXYZRGB>::Ptr outputLineCloud);
void printCloudPoints( const  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
void printCloudPoints ( const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
void statisticLinePointsHeight(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud , double &mean , double &stdv );
double calDistancePoint2Plane(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud , Eigen::Vector3d &normalVector,const Eigen::Vector3d &point , const int index);
void statisticPoints2PlaneMeanStdv(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud , Eigen::Vector3d &normalVector,double &mean , double &stdv);
int calibration_verification_fun(cv::Mat &ir_src_in , cv::Mat &depth_src_in);


#endif // _PLANE_DETECTION_H_