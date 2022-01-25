/*
 * @Author: jiangxinyu
 * @Date: 2021-05-29 10:48:00
 * @LastEditTime: 2021-08-25 15:30:09
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /vscodeTest/main.cpp
 */

 #include "planeDetection.h"

using namespace cv;
using namespace std;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void mySobel(Mat &src,Mat &abs_gradient_x,Mat &abs_gradient_y, Mat &dst3, Mat &direction_)
{
    Mat gradient_x, gradient_y;
    Sobel( src, gradient_x, CV_16S, 1, 0, 3, 1, 1);
    Sobel( src, gradient_y, CV_16S, 0, 1, 3, 1, 1);
    convertScaleAbs( gradient_x,  abs_gradient_x);
    convertScaleAbs( gradient_y, abs_gradient_y );
    Mat abs_x = abs_gradient_x.clone();
    Mat abs_y = abs_gradient_y.clone();
    //合并梯度
    addWeighted(abs_x, 0, abs_y, 1, 0, dst3 );
    
    for (int i = 0; i < gradient_x.rows;i++)
    {
        for(int j =0 ;j < gradient_x.cols;j++)
        {
            direction_.at<int16_t>(i,j) = atan2(gradient_y.at<int16_t>(i,j),gradient_x.at<int16_t>(i,j));
        }
    }
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void myCanny(Mat &src )
{
	blur( src, src, Size(3,3) );
    Canny(src,src,150,100,3);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void findLine(Mat &outputSoble,Mat &directionSobel , Mat &outputCanny , Mat &outputLine)
{
    Mat outputCanny_U8;
    convertScaleAbs( outputCanny,  outputCanny_U8);
    outputLine = outputCanny_U8.clone();
    for (int i = 0; i < outputSoble.rows;i++)
    {
        for(int j =0 ;j < outputSoble.cols;j++)
        {
            if (outputCanny_U8.at<uint8_t>(i,j) != 0 )
            {
                if (outputSoble.at<uint8_t>(i,j) < 150 || directionSobel.at<int16_t>(i,j) != -1 )
                {
                    outputLine.at<uint8_t>(i,j) = 0;//黑色
                }else{
                    outputLine.at<uint8_t>(i,j) = 255; //白色
                }
            }
        }
    }//for

}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief  由直线进行区域生长得到平面
 * 
 * @param outputLine 预先提取的直线点
 * @param src IR图
 * @param depth_src 深度图 
 * @param outputFlat  输出平面
 * @param grayThreshold  与种子点之间的灰度阈值
 * @param depthThreshold  与种子点之间深度阈值
 */
void findFlat(Mat &outputLine ,Mat &src ,Mat &depth_src ,Mat &outputFlat , const int &grayThreshold, const int &depthThreshold)
{
    // make seed 
    std::vector<cv::Point2i> seeds;
    for (int i = 0; i < outputLine.rows;i++)
    {
        for(int j =0 ;j < outputLine.cols;j++)
        {
            if (outputLine.at<uint8_t>(i,j) == 255 && i > 30)
            {
                cv::Point2i seedTmp;
                seedTmp.x = i+1;
                seedTmp.y = j;
                // 在筛选种子时加一个判断，在原始IR图上的灰度值要低
                if (abs(src.at<uint8_t>(seedTmp.x,seedTmp.y) - src.at<uint8_t>(i,j)) < 10
                    && src.at<uint8_t>(seedTmp.x,seedTmp.y) < 125 )
                {
                    seeds.push_back(seedTmp);
                }
            }
        }
    }
    // 做区域生长
    outputFlat = regionGrowFast(src, depth_src, seeds, grayThreshold, depthThreshold);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief 
 * 
 * @param src 
 * @param deprh_src 
 * @param seeds 
 * @param grayThreshold 
 * @param depthThreshold 
 * @return cv::Mat 
 */
cv::Mat regionGrowFast(const cv::Mat &src,  const cv::Mat &depth_src, std::vector<cv::Point2i> &seeds,const int &grayThreshold, const int &depthThreshold )
{
    cv::Mat result = src.clone();
    for (int i = 0; i < result.rows;i++)
    {
        for(int j =0 ;j < result.cols;j++)
        {
            result.at<uint8_t>(i,j) = 0; // 整张图片置成黑色
        }
    }
    if (seeds.empty() || seeds.size() == 0 )
    {
        std::cerr << "There is no seed ... return \n" ;
        return result;
    }
    // grow direction sequenc
    int grow_direction[8][2] = {{-1,-1}, {0,-1}, {1,-1}, {1,0}, {1,1}, {0,1}, {-1,1}, {-1,0}};
    // 开始区域生长  
    while( !seeds.empty())
    {
        //get a seed
        cv::Point2i current_seed = seeds.back();
        seeds.pop_back();
        
        // 遍历八邻域像素
        for( int i = 0; i < 8; ++i )
        {
            cv::Point2i neighbor_seed(current_seed.x + grow_direction[i][0], current_seed.y + grow_direction[i][1]);
            
            // 检查是否超出图片范围
            if(neighbor_seed.x < 0 || neighbor_seed.y < 0 || neighbor_seed.x > (src.rows-1) || (neighbor_seed.y > src.cols -1))
            {
                continue;
            }
            // 得到当前seed灰度值
            int seed_gray = src.at<uint8_t>(current_seed.x, current_seed.y);
            int seed_depth = depth_src.at<uint16_t>(current_seed.x , current_seed.y); // mm
            int neighbor_gray = src.at<uint8_t>(neighbor_seed.x, neighbor_seed.y);
            int neighbor_depth = depth_src.at<uint16_t>(neighbor_seed.x,neighbor_seed.y); // mm
            // 判断是否添加到平面点
            // 如果是黑色且领域和中心的在阈值范围内
            result.at<uint8_t>(current_seed.x,current_seed.y) = 255; 
            if((result.at<uint8_t>(neighbor_seed.x, neighbor_seed.y) == 0) && (abs(neighbor_gray - seed_gray) < grayThreshold)
                && ( abs(seed_depth - neighbor_depth)< depthThreshold)  && neighbor_gray < 125 )
            {
                result.at<uint8_t>(neighbor_seed.x, neighbor_seed.y) = 255; // 设置成白色
                seeds.push_back(neighbor_seed);
            }
        }
    }//while
    return result;
    
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void toFlatPointCloud(const Mat &flat_ , const Mat &depth_ , pcl::PointCloud<pcl::PointXYZRGB> &flatCloud , 
                                        Eigen::Matrix3d &RotationMat_ ,Eigen::Matrix3d &CameraKMat_ ,Eigen::Vector3d &transVec_)
{
    std::vector<Eigen::Vector3d> pixelCoordinates;
    std::vector<Eigen::Vector3d> cameraCoordinates;
    std::vector<Eigen::Vector3d> worldCoordinates;
    // 从图像中获取像素坐标
    for (int i = 0; i < flat_.rows; i++)
    {
        for(int j =0 ;j < flat_.cols; j++)
        {
            if (flat_.at<uint8_t>(i,j) == 255 ) 
            {
                double depth_value = depth_.at<uint16_t>(i,j)/1000.0;
                if ( depth_value != 0 )
                {
                    Eigen::Vector3d uv_tmp(i,j,depth_value);
                    pixelCoordinates.push_back(uv_tmp);
                }
            }
        }
    } // for
    // 像素坐标转成的Camera系下三维点坐标, 通过外参转换到世界坐标系下
    cameraCoordinates.resize(pixelCoordinates.size());
    worldCoordinates.resize(pixelCoordinates.size());
    int i = 0;
    for ( auto uv : pixelCoordinates )
    {
        cameraCoordinates.at(i)[0] = uv[2]*(uv[1]- CameraKMat_(0,2))/CameraKMat_(0,0);
        cameraCoordinates.at(i)[1] = uv[2]*(uv[0]- CameraKMat_(1,2))/CameraKMat_(1,1);
        cameraCoordinates.at(i)[2] = uv[2];
        Eigen::Vector3d worldPoint;
        transformPoints(cameraCoordinates.at(i),RotationMat,transVec,worldPoint);
        worldCoordinates.at(i) = worldPoint;
        i++;
    }
    // 转成点云显示
    flatCloud.resize(worldCoordinates.size());
    int j = 0 ;
    for ( auto &p : flatCloud)
    {
        p.x = worldCoordinates.at(j)[0];
        p.y = worldCoordinates.at(j)[1];
        p.z = worldCoordinates.at(j)[2];
        p.r = 255;
        p.g = 0;
        p.b = 0;
        j++;
    }
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void binaryImage2PointCloud(const Mat &binaryImage, const Mat &depth_ , pcl::PointCloud<pcl::PointXYZRGB> &cloud , 
                                        Eigen::Matrix3d &RotationMat_ ,Eigen::Matrix3d &CameraKMat_ ,Eigen::Vector3d &transVec_)
{
    std::vector<Eigen::Vector3d> pixelCoordinates;
    std::vector<Eigen::Vector3d> cameraCoordinates;
    std::vector<Eigen::Vector3d> worldCoordinates;
    // 从图像中获取像素坐标
    for (int i = 0; i < binaryImage.rows; i++)
    {
        for(int j =0 ;j < binaryImage.cols; j++)
        {
            if (binaryImage.at<uint8_t>(i,j) == 255 ) 
            {
                double depth_value = depth_.at<uint16_t>(i,j)/1000.0;
                if ( depth_value != 0 )
                {
                    Eigen::Vector3d uv_tmp(i,j,depth_value);
                    pixelCoordinates.push_back(uv_tmp);
                }
            }
        }
    } // for
    // 像素坐标转成的Camera系下三维点坐标, 通过外参转换到世界坐标系下
    cameraCoordinates.resize(pixelCoordinates.size());
    worldCoordinates.resize(pixelCoordinates.size());
    int i = 0;
    for ( auto uv : pixelCoordinates )
    {
        cameraCoordinates.at(i)[0] = uv[2]*(uv[1]- CameraKMat_(0,2))/CameraKMat_(0,0);
        cameraCoordinates.at(i)[1] = uv[2]*(uv[0]- CameraKMat_(1,2))/CameraKMat_(1,1);
        cameraCoordinates.at(i)[2] = uv[2];
        Eigen::Vector3d worldPoint;
        transformPoints(cameraCoordinates.at(i),RotationMat,transVec,worldPoint);
        worldCoordinates.at(i) = worldPoint;
        i++;
    }
    // 转成点云显示
    cloud.resize(worldCoordinates.size());
    int j = 0 ;
    for ( auto &p : cloud)
    {
        p.x = worldCoordinates.at(j)[0];
        p.y = worldCoordinates.at(j)[1];
        p.z = worldCoordinates.at(j)[2];
        p.r = 255;
        p.g = 0;
        p.b = 0;
        j++;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void toPointCloud(const Mat &ir_ , const Mat &depth_ , pcl::PointCloud<pcl::PointXYZRGB> &cloud , 
                                        Eigen::Matrix3d &RotationMat_ ,Eigen::Matrix3d &CameraKMat_ ,Eigen::Vector3d &transVec_)
{
    std::vector<Eigen::Vector3d> pixelCoordinates;
    std::vector<Eigen::Vector3d> cameraCoordinates;
    std::vector<Eigen::Vector3d> worldCoordinates;
    // 从图像中获取像素坐标
    for (int i = 0; i < ir_.rows;i++)
    {
        for(int j =0 ;j < ir_.cols;j++)
        {
            double depth_value = depth_.at<uint16_t>(i,j)/1000.0;
            if ( depth_value != 0 )
            {
                Eigen::Vector3d uv_tmp(i,j,depth_value);
                pixelCoordinates.push_back(uv_tmp);
            }
        }
    } // for
    // 像素坐标转成的Camera系下三维点坐标, 通过外参转换到世界坐标系下
    cameraCoordinates.resize(pixelCoordinates.size());
    worldCoordinates.resize(pixelCoordinates.size());
    int i = 0;
    for ( auto uv : pixelCoordinates )
    {
        cameraCoordinates.at(i)[0] = uv[2]*(uv[1] - CameraKMat_(0,2))/CameraKMat_(0,0);
        cameraCoordinates.at(i)[1] = uv[2]*(uv[0] - CameraKMat_(1,2))/CameraKMat_(1,1);
        cameraCoordinates.at(i)[2] = uv[2];
        Eigen::Vector3d worldPoint;
        transformPoints(cameraCoordinates.at(i),RotationMat,transVec,worldPoint);
        worldCoordinates.at(i) = worldPoint;
        i++;
    }
    // 转成点云显示
    cloud.resize(worldCoordinates.size());
    int j = 0 ;
    for ( auto &p : cloud)
    {
        p.x = worldCoordinates.at(j)[0];
        p.y = worldCoordinates.at(j)[1];
        p.z = worldCoordinates.at(j)[2];
        p.r = 0;
        p.g = 255;
        p.b = 0;
        j++;
    }
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////Z
void setCameraKMat(Eigen::Matrix3d &CameraKMat_ ,double fx_,double fy_ ,double cx_,double cy_)
{
    Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
    K << fx_ , 0 , cx_,
            0 , fy_ ,cy_,
            0,0,1;
    CameraKMat_ = K;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////Z
void transformPoints(pcl::PointXYZRGB &point,  const Eigen::Matrix3d &rotationMatrix , const Eigen::Vector3d &transVector)
{
    Eigen::Vector3d pointVec(point.x,point.y,point.z);
    Eigen::Vector3d outPointVec ;
    outPointVec = rotationMatrix*pointVec + transVector;
    point.x = outPointVec[0];
    point.y = outPointVec[1];
    point.z = outPointVec[2];
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////Z
void transformPoints(const Eigen::Vector3d &pointVec,  const Eigen::Matrix3d &rotationMatrix , const Eigen::Vector3d &transVector , Eigen::Vector3d &outputVec)
{
    outputVec = rotationMatrix*pointVec + transVector;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////Z
void readPLY(std::string &plyfilename)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr plyCloud;
    if (pcl::io::loadPLYFile(plyfilename.c_str(), *plyCloud )<0 )
    {
        std::cout << "error";
    }
    pcl::visualization::PCLVisualizer::Ptr viewer2 (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer2->setBackgroundColor (0, 0, 0);
    viewer2->addPointCloud<pcl::PointXYZ> (plyCloud, "sample cloud");
    // viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer2->initCameraParameters ();
    while (!viewer2->wasStopped())
     {
        viewer2->spinOnce(10);
    }
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////Z
int readFile(std::string &filename , unsigned char* data , const int width , const int height)
{
    // 读文件
	char *rawFileName = (char*)filename.c_str();
	FILE *fp = NULL;
	int ret = 0;
	if (NULL == data)
	{
		printf("Fail to calloc buf\r\n");
		return -1;
	}
	if (NULL == (fp = fopen(rawFileName, "rb")))
	{
		printf("Fail to read %s.\r\n", rawFileName);
		return -2;
	}
	ret = fread(data, sizeof(unsigned char)*width*height, 1, fp);
	if (ret != 1)
	{
		printf("Fail to read raw data\r\n");
		return -3;
	}
    return 1;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////Z
int readFile(std::string &filename , uint16_t* data , const int width , const int height)
{
    // 读文件
	char *rawFileName = (char*)filename.c_str();
	FILE *fp = NULL;
	int ret = 0;
	if (NULL == data)
	{
		printf("Fail to calloc buf\r\n");
		return -1;
	}
	if (NULL == (fp = fopen(rawFileName, "rb")))
	{
		printf("Fail to read %s.\r\n", rawFileName);
		return -2;
	}
	ret = fread(data, sizeof(uint16_t)*width*height, 1, fp);
	if (ret != 1)
	{
		printf("Fail to read raw data\r\n");
		return -3;
	}
    return 1;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void findOutlier(Mat &ir_ , Mat &depth_ )
{
    Mat ir2 = ir_.clone();
    for (int i = 0; i < depth_.rows;i++)
    {
        for(int j =0 ;j < depth_.cols;j++)
        {
            if (depth_.at<uint16_t>(i,j) < 50 ) 
            {
                ir2.at<uint8_t>(i,j) = 255;
            }
            else{
                ir2.at<uint8_t>(i,j) = 0;
            }
        }
    } // for
    cvNamedWindow("ir2", 0);
	imshow("ir2", ir2);
	cvWaitKey(0);
	cvDestroyWindow("ir2");
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief RANSAC 求平面方程
 * 
 * @param flatCloud 
 * @param normalVector 
 * @param cloud_plane 
 */
void  calNormalVectorByRansac( pcl::PointCloud<pcl::PointXYZRGB> &flatCloud , Eigen::Vector3d &normalVector , pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane )
{
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr flat_;
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());  //申明存储模型的内点的索引
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());  //申明模型的参数
    seg.setOptimizeCoefficients (true);
    // 以下都是强制性的需要设置的
    seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);   //平面模型
    seg.setMethodType (pcl::SAC_RRANSAC);    //分割平面模型所使用的分割方法
    seg.setDistanceThreshold (0.02);        //设置最小的阀值距离
    seg.setMaxIterations(30000);
    
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
    // std::cout << "RANSAC 平面内点数量为 :  "<< cloud_plane ->size() << "\n"
    //                 <<"平面模型系数为 : \n" 
    //                 << "A = " << coefficients->values[0] << "\n" 
    //                 << "B = " << coefficients->values[1] << "\n" 
    //                 << "C = " << coefficients->values[2] << "\n" 
    //                 << "D ="  << coefficients->values[3] << "\n";
    normalVector << coefficients->values[0] ,coefficients->values[1],coefficients->values[2];
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief 
 * 
 * @param lineCloud 
 * @param directionVector 
 * @param outputLineCloud 
 */
void calLineVectorByRansac(pcl::PointCloud<pcl::PointXYZRGB> &lineCloud , Eigen::Vector3d &directionVector , pcl::PointCloud<pcl::PointXYZRGB>::Ptr outputLineCloud)
{
    // 1 分割直线
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());  //申明存储模型的内点的索引
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PARALLEL_LINE);
    seg.setMethodType(pcl::SAC_RANSAC);
    const Eigen::Vector3f axis(1,0,0); // 
    const double eps  = 2;
    seg.setAxis(axis);
    seg.setEpsAngle(eps);
    seg.setMaxIterations(30000);
    seg.setDistanceThreshold(0.01);
    seg.setInputCloud(lineCloud.makeShared());
    seg.segment(*inliers,*coefficients);

    // 2 使用索引提取器提取直线点云
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud(lineCloud.makeShared());
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*outputLineCloud);

    // 3 打印信息
    // std::cout << "RANSAC 直线内点数量为 : " << outputLineCloud->size() << "\n"
    //                 << "直线方向 X = " << coefficients->values[3] << "\n"
    //                 << "直线方向 Y = " << coefficients->values[4] << "\n"
    //                 << "直线方向 Z = " << coefficients->values[5] << "\n"
    //                 << "直线上某一点 ：" << "(" << coefficients->values[0] << "," << coefficients->values[1] << "," << coefficients->values[2] <<")\n";
    // directionVector << coefficients->values[3], coefficients->values[4],coefficients->values[5];
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void calNormalVecByPCA(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud )
{
    Eigen::Matrix<double, 4, 1> centroid;
    pcl::compute3DCentroid(*cloud, centroid);   // 计算质心
    Eigen::Matrix<double, 3, 3> convariance_matrix;  // 协方差矩阵
    pcl::computeCovarianceMatrix(*cloud, centroid, convariance_matrix);//计算协方差
    Eigen::Matrix3d eigenVectors;
    Eigen::Vector3d eigenValues;
    pcl::eigen33(convariance_matrix, eigenVectors, eigenValues);
    // 查找最小特征值的位置
    Eigen::Vector3d::Index minRow, minCol;
    eigenValues.minCoeff(&minRow, &minCol);
    // 获取点云质心
    double X = centroid(0);
    double Y = centroid(1);
    double Z = centroid(2);
    // 获取平面方程：AX+BY+CZ+D = 0的系数
    double A = eigenVectors(0, minRow);
    double B = eigenVectors(1, minRow);
    double C = eigenVectors(2, minRow);
    double D = -(A * X + B * Y + C * Z);

    std::cout << "\n拟合平面内点数量为 : " << cloud->size() << "\n";
    std::cout << "\nPCA 平面内点模型系数为 : "
        << "A = " << A << "," 
        << "B = " << B << ","
        << "C = " << C << "," 
        << "D = " << D << "\n";
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief 由PCA计算点云主方向
 * 
 * @param cloud 
 * @param mainDirectoin 
 * @param normalDirection 
 */
void calNormalVecByPCA(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud , Eigen::Vector3d &mainDirectoin , Eigen::Vector3d &normalDirection )
{
    Eigen::Matrix<double, 4, 1> centroid;
    pcl::compute3DCentroid(*cloud, centroid);   // 计算质心
    Eigen::Matrix<double, 3, 3> convariance_matrix;  // 协方差矩阵
    pcl::computeCovarianceMatrix(*cloud, centroid, convariance_matrix);//计算协方差

    Eigen::Matrix3d eigenVectors;
    Eigen::Vector3d eigenValues;
    pcl::eigen33(convariance_matrix, eigenVectors, eigenValues);
    // 查找最小特征值的位置
    Eigen::Vector3d::Index maxRow, maxCol;
    eigenValues.maxCoeff(&maxRow, &maxCol);
    mainDirectoin << eigenVectors(0, maxRow) , eigenVectors(1, maxRow) , eigenVectors(2, maxRow);
    mainDirectoin.normalize();
    std::cout << "\n拟合直线内点数量为 : " << cloud->size() << "\n";
    std::cout << "\nPCA直线主方向为 : " <<"(" << mainDirectoin[0] << "," << mainDirectoin[1] << "," << mainDirectoin[2] << ")\n";
    Eigen::Vector3d::Index minRow, minCol;
    eigenValues.minCoeff(&minRow, &minCol);
    // 获取点云质心
    double X = centroid(0);
    double Y = centroid(1);
    double Z = centroid(2);
    // 获取平面方程：AX+BY+CZ+D = 0的系数
    double A = eigenVectors(0, minRow);
    double B = eigenVectors(1, minRow);
    double C = eigenVectors(2, minRow);
    double D = -(A * X + B * Y + C * Z);
    normalDirection << A,B,C;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void printCloudPoints( const  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    if (cloud->size() == 0 )
    {
        return;
    }
    std::cout << "Point Cloud Size = " << cloud->points.size() << "\n";
    for (auto p : *cloud)
    {
        std::cout << p.x  << "," << p.y << "," << p.z <<"\n";
    }
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void printCloudPoints ( const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    std::cout << "Point Cloud Size = " << cloud->points.size() << "\n";
    for (auto p : *cloud)
    {
        std::cout << p.x  << "," << p.y << "," << p.z <<"\n";
    }
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 统计直线上的点的高度
void statisticLinePointsHeight(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud , double &mean , double &stdv )
{
    mean = 0;
    stdv = 0;
    if ( cloud->empty() || cloud->size() < 10 )
    {
        std::cerr << "The line points number < 10 ... \n";
    }
    // 计算
    int pointNumber = cloud->size();
    double sum = 0;
    for ( auto p : (*cloud) )
    {
        sum += p.y;
    }
    mean = (double) sum/pointNumber;
    // 
    double tmp = 0;
    for (auto p : (*cloud))
    {
        tmp += std::pow((p.y - mean),2);
    }
    stdv = std::sqrt((tmp/(pointNumber -1)));
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief 计算平面的内点到平面的距离和标准差
 * 
 * @param cloud 
 * @param normalVector 
 * @param mean 
 * @param stdv 
 */
void statisticPoints2PlaneMeanStdv(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud , Eigen::Vector3d &normalVector,double &mean , double &stdv)
{
    Eigen::Vector3d normalVec = normalVector;
    mean = 0 ; stdv = 0;
    if ( cloud->empty() || cloud->size() < 10 )
    {
        std::cerr << "plane size < 10 ..." << "\n";
    }
    int cloudNumber = cloud->size()-1;
    int index = cloudNumber;
    int i = 0 ;
    double sumDistance = 0;
    double sumTmp = 0 ;
    std::vector<double> distances;
    for ( auto p : (*cloud))
    {
        double tmpx = p.x;
        double tmpy = p.y;
        double tmpz = p.z;
        Eigen::Vector3d curPoint(tmpx,tmpy,tmpz);
        auto curDistance =  calDistancePoint2Plane(cloud,normalVec,curPoint,index);
        distances.emplace_back(curDistance);
        sumDistance += curDistance;
    }
    mean = (double) sumDistance/cloudNumber;

    for (auto dis : distances)
    {
        sumTmp += std::pow((dis-mean),2);
    }
    stdv = std::sqrt(sumTmp/(cloudNumber-1));
    std::vector<double>().swap(distances);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double calDistancePoint2Plane(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud , Eigen::Vector3d &normalVector,const Eigen::Vector3d &point , const int index)
{
    double x = cloud->points[index].x;
    double y = cloud->points[index].y;
    double z = cloud->points[index].z;
    Eigen::Vector3d anotherPoint(x,y,z);
    if( point == anotherPoint )
    {
        return 0;
    }
    Eigen::Vector3d tmpDirection( point[0]-anotherPoint[0], point[1]-anotherPoint[1],point[2]-anotherPoint[2]);
    auto result = (tmpDirection.transpose()*normalVector) / normalVector.norm();
    // std::cout << "result  = " << result(0,0) << "\n";
    return result(0,0);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief 
 * 
 * @param ir_src_in 
 * @param depth_src_in 
 * @return int 
 */
int calibration_verification_fun(cv::Mat &ir_src_in , cv::Mat &depth_src_in )
{
    // 数据输入
    std::cout << "calibration_verification_fun  :::: " << std::endl;
    Mat src = ir_src_in.clone();
    Mat depth_src = depth_src_in.clone();

#ifdef DISPLAY
    cvNamedWindow("src", 0);
	imshow("src", src);
	cvWaitKey(0);
    Mat depth_src_show;
    cv::convertScaleAbs(depth_src,depth_src_show);
    cvNamedWindow("depth_src_show", 0);
	imshow("depth_src_show", depth_src_show);
	cvWaitKey(0);
    cvDestroyWindow("depth_src_show");
	cvDestroyWindow("src");
#endif


////////////////////////////////////////////////////////////////////////
    //  Canny 算子求边缘
    Mat outputCanny = src.clone();
    myCanny(outputCanny);

#ifdef DISPLAY    
    cvNamedWindow("Canny", 0);
	imshow("Canny", outputCanny);
	cvWaitKey(0);
	cvDestroyWindow("Canny");
#endif

////////////////////////////////////////////////////////////////////////
    //  Sobel算子求边缘
    Mat img = src.clone();
    GaussianBlur( img, img, Size(3,3), 0, 0, BORDER_DEFAULT );
    Mat abs_gradient_x;
    Mat abs_gradient_y;
    Mat dst3;
    Mat direction_(Size(width, height), CV_16S);
    mySobel(img,abs_gradient_x,abs_gradient_y,dst3,direction_);

////////////////////////////////////////////////////////////////////////
    // 显示使用Sobel算子检测出来的边缘(Y)
 #ifdef DISPLAY   
    cvNamedWindow("abs_gradient_y", 0);
	imshow("abs_gradient_y", abs_gradient_y);
	cvWaitKey(0);
#endif
/////////////////////////////////////////////////////////////////////////////
    /*
    *   使用梯度的方向和幅值进一步提上边缘
    */
    Mat outputSobel;
    outputSobel = abs_gradient_y.clone();
    // 遍历图片
    for (int i = 0; i < outputSobel.rows;i++)
    {
        for(int j =0 ;j < outputSobel.cols;j++)
        {
            if ( outputSobel.at<uint8_t>(i,j) >180 && direction_.at<int16_t>(i,j) == -1)
            {
                continue;
            }
            outputSobel.at<uint8_t>(i,j) = 0;
        }
    }
#ifdef DISPLAY
    cvNamedWindow("outputSobel", 0);
    imshow("outputSobel", outputSobel);
	cvWaitKey(0);
#endif

////////////////////////////////////////////////////////////////////////
    // 使用Canny和Sobel输出的图像进一步找共同的像素->提取出特定的直线
    Mat outputLine;
    findLine(outputSobel,direction_,outputCanny,outputLine);
    
#ifdef DISPLAY    
    cvNamedWindow("outputLine", 0);
	imshow("outputLine", outputLine);
	cvWaitKey(0);
	cvDestroyWindow("outputLine");
#endif
////////////////////////////////////////////////////////////////////////

    // 基于区域生长求出像素平面
    Mat outputFlat;
    Mat src2 = src.clone();
    const int grayThreshold = 3;
    const int depthThreshold = 15;//mm
    findFlat(outputLine,src2,depth_src_in,outputFlat,grayThreshold,depthThreshold);

 #ifdef DISPLAY
    cvNamedWindow("outputFlat", 0);
    imshow("outputFlat", outputFlat);
	cvWaitKey(0);
 #endif
 ////////////////////////////////////////////////////////////////////////
    // 将像素平面的点转成点云

    pcl::PointCloud<pcl::PointXYZRGB> irCloud;
    toPointCloud(src,depth_src,irCloud,RotationMat,CameraKMat,transVec);
    std::cout << "\nValid Points' number = " << irCloud.size() << "\n";

    pcl::PointCloud<pcl::PointXYZRGB> lineCloud;
    binaryImage2PointCloud(outputLine,depth_src,lineCloud,RotationMat,CameraKMat,transVec);
    std::cout << "\nLine Points' number = " << lineCloud.size() << "\n";

    pcl::PointCloud<pcl::PointXYZRGB> flatCloud;
    toFlatPointCloud(outputFlat, depth_src ,flatCloud,RotationMat,CameraKMat,transVec);
    std::cout << "\nFlat Points' number = " << flatCloud.size() << "\n";
    
 ////////////////////////////////////////////////////////////////////////
    // 点云显示
#ifdef PCL_DISPLAY
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->initCameraParameters ();
    int v1(0);
    viewer->createViewPort (0.0, 0.0, 0.5, 1.0, v1);
    viewer->setBackgroundColor (0, 0, 0, v1);
    // viewer->addText ("Radius: 0.01", 10, 10, "v1 text", v1);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb (irCloud.makeShared());
    viewer->addPointCloud<pcl::PointXYZRGB> (irCloud.makeShared(), rgb, "sample cloud1", v1);
    viewer->addCoordinateSystem(0.3);
    int v2(0);
    viewer->createViewPort (0.5, 0.0, 1.0, 1.0, v2);
    viewer->setBackgroundColor (0,0,0, v2);
    // viewer->addText ("Radius: 0.1", 10, 10, "v2 text", v2);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color (flatCloud.makeShared(), 255, 0, 0);
    viewer->addPointCloud<pcl::PointXYZRGB> (flatCloud.makeShared(), single_color, "sample cloud2", v2);
    viewer->addCoordinateSystem(0.3);
    while ( !viewer->wasStopped () )
    {
        viewer->spinOnce(10);
    }
#endif
////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////

    // Ransac 求平面内点
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB>);
    Eigen::Vector3d normalVec;
    // calNormalVectorByRansac(flatCloud,normalVec,cloud_plane);
    // PCA 求平面内点法向量
    // calNormalVecByPCA(cloud_plane);
    // printCloudPoints(cloud_plane.makeShared());

    // Ransac 求直线内点
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_line(new pcl::PointCloud<pcl::PointXYZRGB>);
    Eigen::Vector3d lineVec;
    Eigen::Vector3d normalLineVec;
    calLineVectorByRansac(lineCloud,lineVec,cloud_line);
    calNormalVecByPCA(cloud_line,lineVec,normalLineVec);

    // 统计直线点的y值的均值和标准差
    double y_mean , y_stdv;
    statisticLinePointsHeight(cloud_line,y_mean,y_stdv);
    //  统计拟合平面上的点到拟合平面的距离
    double plane_mean, plane_stdv;
    // statisticPoints2PlaneMeanStdv(flatCloud.makeShared(),normalVec,plane_mean,plane_stdv);
    
    // 验证外参是否正确
    Eigen::Vector3d normalVecXOZ(0.0,1.0,0.0);
    Eigen::Vector3d lineVecAxisX (1.0,0.0, 0.0);
    normalVec.normalize();
    lineVec.normalize();
    // 计算平面法向量和XOZ平面法向量是否垂直
    std::cout << "\n拟合直线点Y值的均值和标准差 : " << "y_mean = " << y_mean << ", y_stdv = " << y_stdv << "\n";
    // std::cout << "\n平面点到拟合平面距离的均值和标准差为 : " << "plane_mean = " << plane_mean << ",plane_stdv = " << plane_stdv << "\n";

    // std::cout << "\n拟合平面的法向量与拟合直线方向向量的内积为 : " << normalVec.transpose() * lineVec << "\n";
    // std::cout << "\n拟合平面法向量与XOZ平面法向量内积为 : " << normalVec.transpose()*normalVecXOZ<< "\n";
    std::cout << "\n拟合直线与XOZ平面法向量内积为 : " << lineVec.transpose() * normalVecXOZ << "\n" << "\n";

    // auto angle_line_x = std::atan2(lineVec.cross(lineVecAxisX).norm(),lineVec.transpose()*lineVecAxisX);
    // std::cout << "拟合直线与X轴夹角为 : " << RAD2DEG(angle_line_x) << "° \n" << "\n"; 
    


#ifdef PCL_DISPLAY
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2 (new pcl::visualization::PCLVisualizer ("3D Viewer2"));
    viewer2->initCameraParameters ();
    int v3(0);
    viewer2->createViewPort (0.0, 0.0, 0.5, 1.0, v3);
    viewer2->setBackgroundColor (0, 0, 0, v3);
    // viewer->addText ("Radius: 0.01", 10, 10, "v1 text", v1);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb2 (cloud_plane);
    viewer2->addPointCloud<pcl::PointXYZRGB> (cloud_plane, rgb2, "sample cloud1", v3);
    viewer2->addCoordinateSystem(0.3);
    while ( !viewer2->wasStopped () )
    {
        viewer2->spinOnce(10);
    }
#endif    

#ifdef DISPLAY    
    // destroy  window

    cvDestroyWindow("outputSobel");
    cvDestroyWindow("abs_gradient_y");
    cvDestroyWindow("outputFlat");
#endif    
	return 1;
}