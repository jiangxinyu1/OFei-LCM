/*
 * @Author: jiangxinyu
 * @Date: 2021-08-09 16:17:10
 * @LastEditTime: 2021-09-28 11:33:20
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /360-0366-demo/imagesHandler.cpp
 */


extern "C" {
    #include "camerademo.h"
    #include "calibration.h"
    #include "DepthMapWrapper.h"
}

#include "imagesHandler.h"

extern uint8_t raw12_img[336*1557];
extern char ir_image[224*114];
extern char depth_image[224*114*2];
extern char pcloud_image[224*114*sizeof(pc_pkt_t)];
extern char depth_data[224*114*sizeof(depth_data_pkt_t)];
extern unsigned int pt_count;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
inline int64_t getTimestamp()
{
    struct timespec t;
    t.tv_sec = t.tv_nsec = 0;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)(t.tv_sec * 1000L + t.tv_nsec / 1000000L);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void imagesHandler::pubDepthImage( char* image , const int width_, const int height_,const int size_ , const std::string &type_)
{
    uint16_t * image_ = (uint16_t*)image;
    lcm_sensor_msgs::Image lcm_image;
    auto ir_t  = timeStamp_;
    lcm_ros::Time irTime;
    irTime.sec = int64_t(ir_t);
    irTime.nsec = (ir_t - irTime.sec) * 1e9;
    lcm_image.header.stamp=irTime;
    lcm_image.width = (int32_t)width_;
    lcm_image.height = (int32_t)height_;
    lcm_image.n_data = (int32_t)size_;
    lcm_image.data.resize(lcm_image.n_data);
    lcm_image.encoding = "16UC1";
    // memcpy(lcm_image.data.data(), image_, lcm_image.n_data*sizeof(uint16_t));
    for ( auto i = 0 ; i < size_; i++ )
    {
        lcm_image.data.at(i) = (uint16_t)(image_[i] & 0X1FFF );
    }
    // std::cout << "lcm_image.data.depth = " << lcm_image.data.at(224*57+112) << "\n";
    const std::string  topicName = type_ + "_image";
    _lcm->publish(topicName, &lcm_image);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void imagesHandler::pubIRImage( char* image , const int width_, const int height_,const int size_ , const std::string &type_)
{
    lcm_sensor_msgs::Image lcm_image;
    auto ir_t  = timeStamp_;
    lcm_ros::Time irTime;
    irTime.sec = int64_t(ir_t);
    irTime.nsec = (ir_t - double(irTime.sec)) * 1e9;
    lcm_image.header.stamp=irTime;//时间戳保持和其他话题消息一致
    lcm_image.width = (int32_t)width_;
    lcm_image.height = (int32_t)height_;
    lcm_image.n_data = (int32_t)size_;
    lcm_image.data.resize(lcm_image.n_data);
    lcm_image.encoding = "16UC1";
    for ( auto i = 0 ; i < size_; i++ )
    {
        lcm_image.data.at(i) = (uint16_t)(image[i]);
    }
    const std::string  topicName = type_ + "_image";
    _lcm->publish(topicName, &lcm_image);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void imagesHandler::pubPointCloud(char* pcloud_image ,const int size_ , const std::string &type_ )
{
    pc_pkt_t* pointcloud = (pc_pkt_t*)pcloud_image;
    lcm_sensor_msgs::PointCloud cloud;
    auto time_t  = timeStamp_;
    lcm_ros::Time pcTime;
    pcTime.sec = int64_t(time_t);
    pcTime.nsec = (time_t - double(pcTime.sec)) * 1e9;
    cloud.header.stamp=pcTime;//时间戳保持和其他话题消息一致
    cloud.n_points = size_;
    cloud.header.frame_id = "base_link";
    cloud.channels.resize(1);
    cloud.channels[0].name = "intensity";
    cloud.channels[0].values.resize(size_);
    for (auto i = 0 ; i < size_; i++ )
    {
        lcm_geometry_msgs::Point32 pointTmp;
        pointTmp.x = pointcloud[i].X;
        pointTmp.y = pointcloud[i].Y;
        pointTmp.z = pointcloud[i].Z;
        cloud.points.push_back(pointTmp);
        cloud.channels[0].values[i] = pointcloud[i].c;
    }
    // std::cout << "pointcloud.z = " << pointcloud[224*57+112].Z << "\n";
    const std::string  topicName = type_ ;
    _lcm->publish(topicName, &cloud);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void imagesHandler::getTime()
{
    timeStamp_  = getTimestamp();
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int pubImagesRealtime(struct camera_hal *camera ,imagesHandler &handler)
{
	enum v4l2_buf_type type;
	struct timeval tv;
	struct v4l2_buffer buf;
	int i = 0, j = 0, ret = 0, np = 0;
	fd_set fds;

	/* stream on */
	if (camera->driver_type == V4L2_CAP_VIDEO_CAPTURE_MPLANE) {
		type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	} else {
		type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	}

	if (ioctl(camera->videofd, VIDIOC_STREAMON, &type) == -1) {
		camera_err(" VIDIOC_STREAMON error! %s\n",strerror(errno));
		ret = -1;
		goto EXIT;
	} else {
		camera_print(" stream on succeed\n");
	}

	memset(&buf, 0, sizeof(struct v4l2_buffer));
	if (camera->driver_type == V4L2_CAP_VIDEO_CAPTURE_MPLANE) {
		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	}
	else {
		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	}

	buf.memory = V4L2_MEMORY_MMAP;
	if (camera->driver_type == V4L2_CAP_VIDEO_CAPTURE_MPLANE) {
		buf.length = camera->nplanes;
		buf.m.planes = (struct v4l2_plane *)calloc(camera->nplanes, sizeof(struct v4l2_plane));
	}

	DepthMapWrapperStartCapture();
	DepthMapWrapperSetExposureTime(100);

	FD_ZERO(&fds);
	FD_SET(camera->videofd, &fds);
    
	while (handler.isLcmOk())
	{
        get_protection_data();
		camera_print(" capture num is [%d]\n",np);
		tv.tv_sec = 2;
		tv.tv_usec = 0;

		ret = select(camera->videofd+1,&fds,NULL,NULL,&tv);
		if (ret == -1) {
			camera_err(" select error\n");
			break;
		} else if (ret == 0) {
			camera_err(" select timeout,end capture thread!\n");
			ret = -1;
			break;
		}

		/* Dequeue buf */
		if (ioctl(camera->videofd, VIDIOC_DQBUF, &buf) == 0) {
			camera_dbg("*****DQBUF[%d] FINISH*****\n",buf.index);
		} else {
			camera_err("****DQBUF FAIL*****\n");
			ret = -1;
			break;
		}

		/* Save Raw Data */
		// memset(source_data_path, 0, sizeof(source_data_path));
		// sprintf(source_data_path, "%s/raw_%d.raw", camera->save_path,  np+1);
		// save_frame_to_file(source_data_path, camera->buffers[buf.index].start[0], camera->buffers[buf.index].length[0]-224*2);
		/* Decode Raw Data */
        // timespec time1;
        // timespec time2;
		// clock_gettime(CLOCK_REALTIME, &time1);	
		DepthMapWrapperProcessFrame((char*)(camera->buffers[buf.index].start[0]), ir_image, depth_image, pcloud_image, depth_data, &pt_count);
		// clock_gettime(CLOCK_REALTIME, &time2);	
		// auto temp = (time2.tv_nsec - time1.tv_nsec) / 1000000;
		// printf("\n	time = %f ms		\n",temp);

                DepthMapWrapperGetExposureTime();

		/* Save others */
        handler.getTime();
        handler.pubDepthImage(depth_image,224,114,224*114,"depth");
        handler.pubIRImage(ir_image,224,114,224*114,"ir");
        handler.pubPointCloud(pcloud_image,pt_count,"pointcloud_3d");

		/* Queue buf */
		if (ioctl(camera->videofd, VIDIOC_QBUF, &buf) == 0) {
			camera_dbg("************QBUF[%d] FINISH**************\n",buf.index);
		} else {
			camera_err("*****QBUF FAIL*****\n");
			ret = -1;
			break;
		}

		np++;

usleep(100000);
	}
	
	camera_print("\n Capture thread finish\n");
	DepthMapWrapperTearDown();

	/* stream off */
	if(ioctl(camera->videofd, VIDIOC_STREAMOFF, &type) == -1) {
		camera_err(" VIDIOC_STREAMOFF error! %s\n",strerror(errno));
		ret = -1;
	}

EXIT:
	/* munmap camera->buffers */
	if (camera->driver_type == V4L2_CAP_VIDEO_CAPTURE_MPLANE) {
		for (i = 0; i < camera->buf_count; i++) {
			for (j = 0; j < camera->nplanes; j++) {
				munmap(camera->buffers[i].start[j], camera->buffers[i].length[j]);
			}
		}
	} else {
		for(i=0; i<camera->buf_count; i++) {
			munmap(camera->buffers[i].start[0],camera->buffers[i].length[0]);
		}
	}
	/* free camera->buffers and close camera->videofd */
	camera_print(" close /dev/video%d\n", camera->camera_index);
	if (camera->driver_type == V4L2_CAP_VIDEO_CAPTURE_MPLANE) {
		free(buf.m.planes);
	}

	free(camera->buffers);
	close(camera->videofd);
	return ret;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////