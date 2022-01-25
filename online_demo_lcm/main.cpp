/*
 * @Author: jiangxinyu
 * @Date: 2021-08-09 12:01:24
 * @LastEditTime: 2022-01-13 21:52:07
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /360-0366-demo/main.cpp
 */


// #ifdef __cpluscplus
extern "C" {
// #endif

#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <time.h>
#include <linux/videodev2.h>
#include <pthread.h>
#include <semaphore.h>
#include <ctype.h>
#include <errno.h>
#include <stdio.h>
#include "camerademo.h"
#include "./common.h"
#include "DepthMapWrapper.h"
#include "calibration.h"

// #ifdef __cpluscplus
}
// #endif

#include "imagesHandler.h"

// 2种模式2选1
// #define USE_512M
#define USE_1T



extern int capture_photo(struct camera_hal *camera);
extern int pubImagesRealtime(struct camera_hal *camera);

uint8_t raw12_img[336*639];
char ir_image[224*70];
char depth_image[224*70*2];
char pcloud_image[224*70*sizeof(pc_pkt_t)];
char depth_data[224*70*sizeof(depth_data_pkt_t)];
unsigned int pt_count;
 
int camera_dbg_en = 0;

int main(int argc,char **argv)
{

#ifdef USE_512M
    system("media-ctl -d /dev/media1 --set-v4l2 '\"rkisp-isp-subdev\":0[crop:(0,0)/224x639]'");
    system("media-ctl -d /dev/media1 --set-v4l2 '\"rkisp-isp-subdev\":0[fmt:SBGGR12_1X12/224x639]'");
    system("media-ctl -d /dev/media1 --set-v4l2 '\"rkisp-isp-subdev\":2[crop:(0,0)/224x639]'");
    system("media-ctl -d /dev/media1 --set-v4l2 '\"rkisp-isp-subdev\":2[fmt:SBGGR12_1X12/224x639]'");
#endif // USE_512M

	struct camera_hal camera;
	char video_path[20];
	struct v4l2_capability cap;      /* Query device capabilities */
	struct v4l2_format fmt;          /* try a format */
	//struct v4l2_input inp;           /* select the current video input */
	//struct v4l2_streamparm parms;    /* set streaming parameters */
	struct v4l2_requestbuffers req;  /* Initiate Memory Mapping or User Pointer I/O */
	struct v4l2_buffer buf;          /* Query the status of a buffer */

	int n_buffers = 0;
	int i = 0;

        TOFMODE usecase[4] = {
        TOFMODE_STERO_5FPS , 
        TOFMODE_STERO_10FPS ,
        TOFMODE_STERO_15FPS ,
        TOFMODE_STERO_30FPS ,
        };

	/* default settings */
	memset(&camera, 0, sizeof(struct camera_hal));
	memset(video_path, 0, sizeof(video_path));

#ifdef USE_512M
    camera.camera_index = 5;
    std::cout << "USE_512M , "<< "camera_index = " << camera.camera_index << "\n";
#endif

#ifdef USE_1T
    camera.camera_index = 0;
    std::cout << "USE_1T ," << "camera_index = " << camera.camera_index << "\n";
#endif
	
	camera.win_width = 224;
	camera.win_height = 639;
	camera.photo_num = 5;
	camera.pixelformat = V4L2_PIX_FMT_SBGGR12;

	memcpy(camera.save_path,"/userdata/camera", sizeof("/userdata/camera"));

	camera_print("**********************************************************\n");
	camera_print("*                                                        *\n");
	camera_print("*              this is tof camera test.                  *\n");
	camera_print("*                                                        *\n");
	camera_print("**********************************************************\n");

	/* 1.open /dev/videoX node */
	sprintf(video_path, "%s%d", "/dev/video", camera.camera_index);
	camera_print("**********************************************************\n");
	camera_print(" open %s!\n", video_path);
	camera_print("**********************************************************\n");

	camera.videofd = open(video_path, O_RDWR, 0);
	if (camera.videofd < 0) {
		camera_err(" open %s fail!!!\n", video_path);
		return -1;
	}

	/* 2.Query device capabilities */
	memset(&cap, 0, sizeof(cap));
	if (ioctl(camera.videofd,VIDIOC_QUERYCAP, &cap) < 0) {
		camera_err(" Query device capabilities fail!!!\n");
	} else {
		//CAP变量中包含了该设备的能力信息
		camera_dbg(" Querey device capabilities succeed\n");
		camera_dbg(" cap.driver=%s\n",cap.driver);
		camera_dbg(" cap.card=%s\n",cap.card);
		camera_dbg(" cap.bus_info=%s\n",cap.bus_info);
		camera_dbg(" cap.version=0x%08x\n",cap.version);
		camera_dbg(" cap.capabilities=0x%08x\n",cap.capabilities);
	}

	if((cap.capabilities & (V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_VIDEO_CAPTURE_MPLANE)) <= 0) {
		camera_err(" The device is not supports the Video Capture interface!!!\n");
		close(camera.videofd);
		return -1;
	}

	if (cap.capabilities & V4L2_CAP_VIDEO_CAPTURE_MPLANE) {
		camera.driver_type = V4L2_CAP_VIDEO_CAPTURE_MPLANE;
		camera_dbg(" Camera Support VIDEO_CAPTURE_MPLANE\n");
	} else if(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) {
		camera.driver_type = V4L2_CAP_VIDEO_CAPTURE;
		camera_dbg(" Camera Support VIDEO_CAPTURE\n");
	} else {
		camera_err(" %s is not a capture device.\n", video_path);
		close(camera.videofd);
		return -1;
	}

	camera_print("**********************************************************\n");
	camera_print(" The path to data saving is %s.\n",camera.save_path);
	camera_print(" The number of captured photos is %d.\n",camera.photo_num);

	#if 0
	/* 3.select the current video input */
	inp.index = 0;
	inp.type = V4L2_INPUT_TYPE_CAMERA;
	if (ioctl(camera.videofd,VIDIOC_S_INPUT,&inp) < 0) {
		camera_err(" VIDIOC_S_INPUT failed! s_input: %d\n",inp.index);
		close(camera.videofd);
		return -1;
	}
	#endif

	get_calibration_data(DEFAULT_I2C_BUS, PMD_SPC_FOLDER); //读取标定文件


	#if 0
	/* 4.set streaming parameters */
	memset(&parms, 0, sizeof(struct v4l2_streamparm));
	if (camera.driver_type == V4L2_CAP_VIDEO_CAPTURE_MPLANE) {
		parms.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	} else {
		parms.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	}
	parms.parm.capture.timeperframe.numerator = 1;
	parms.parm.capture.timeperframe.denominator = 10;

	if (ioctl(camera.videofd,VIDIOC_S_PARM,&parms) < 0) {
		camera_err(" Setting streaming parameters failed, numerator:%d denominator:%d\n",
			parms.parm.capture.timeperframe.numerator,
			parms.parm.capture.timeperframe.denominator);//设置视频的帧率
		close(camera.videofd);
		return -1;
	}

	/* 5.get streaming parameters */
	memset(&parms,0,sizeof(struct v4l2_streamparm));
	if (camera.driver_type == V4L2_CAP_VIDEO_CAPTURE_MPLANE) {
		parms.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	} else {
		parms.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	}

	if (ioctl(camera.videofd,VIDIOC_G_PARM,&parms) < 0) {
		camera_err(" Get streaming parameters failed!\n");
	} else{
		camera_print(" Camera capture framerate is %u/%u\n",
			parms.parm.capture.timeperframe.denominator,
			parms.parm.capture.timeperframe.numerator);
	}
	#endif


	/* 6.set the data format */
	memset(&fmt, 0, sizeof(struct v4l2_format));
	if (camera.driver_type == V4L2_CAP_VIDEO_CAPTURE_MPLANE) {
		fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
		fmt.fmt.pix_mp.width = camera.win_width;
		fmt.fmt.pix_mp.height = camera.win_height;
		fmt.fmt.pix_mp.pixelformat = camera.pixelformat;
		fmt.fmt.pix_mp.field = V4L2_FIELD_NONE;
	} else {
		fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		fmt.fmt.pix.width = camera.win_width;
		fmt.fmt.pix.height = camera.win_height;
		fmt.fmt.pix.pixelformat = camera.pixelformat;
		fmt.fmt.pix.field = V4L2_FIELD_NONE;
	}

	if (ioctl(camera.videofd, VIDIOC_S_FMT, &fmt) < 0) {
		camera_err(" setting the data format failed!\n");//设置当前格式
		close(camera.videofd);
		return -1;
	}

	if (camera.driver_type == V4L2_CAP_VIDEO_CAPTURE_MPLANE) {
		if(camera.win_width != fmt.fmt.pix_mp.width || camera.win_height != fmt.fmt.pix_mp.height) {
			camera_err("MPLane does not support %u * %u\n", camera.win_width, camera.win_height);
		}

		camera.win_width = fmt.fmt.pix_mp.width;
		camera.win_height = fmt.fmt.pix_mp.height;
		camera_print(" VIDIOC_S_FMT succeed\n");
		camera_print(" fmt.type = %d\n",fmt.type);
		camera_print(" fmt.fmt.pix.width = %d\n",fmt.fmt.pix_mp.width);
		camera_print(" fmt.fmt.pix.height = %d\n",fmt.fmt.pix_mp.height);
		camera_print(" fmt.fmt.pix.pixelformat = %s\n", get_format_name(fmt.fmt.pix.pixelformat));
		camera_print(" fmt.fmt.pix.field = %d\n",fmt.fmt.pix_mp.field);

		if (ioctl(camera.videofd, VIDIOC_G_FMT, &fmt) < 0) {
			camera_err(" get the data format failed!\n");//查看当前格式
		}

		camera.nplanes = fmt.fmt.pix_mp.num_planes;
	} else {
		if(camera.win_width != fmt.fmt.pix.width || camera.win_height != fmt.fmt.pix.height) {
			camera_err(" does not support %u * %u\n", camera.win_width, camera.win_height);
		}

		camera.win_width = fmt.fmt.pix.width;
		camera.win_height = fmt.fmt.pix.height;
		camera_print(" VIDIOC_S_FMT succeed\n");
		camera_print(" fmt.type = %d\n",fmt.type);
		camera_print(" fmt.fmt.pix.width = %d\n",fmt.fmt.pix.width);
		camera_print(" fmt.fmt.pix.height = %d\n",fmt.fmt.pix.height);
		camera_print(" fmt.fmt.pix.pixelformat = %s\n",get_format_name(fmt.fmt.pix.pixelformat));
		camera_print(" fmt.fmt.pix.field = %d\n",fmt.fmt.pix.field);
	}

	/* 7.Initiate Memory Mapping or User Pointer I/O */
	memset(&req, 0, sizeof(struct v4l2_requestbuffers));
	req.count = 3;
	if (camera.driver_type == V4L2_CAP_VIDEO_CAPTURE_MPLANE) {
		req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	} else {
		req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	}

	req.memory = V4L2_MEMORY_MMAP;
	if (ioctl(camera.videofd, VIDIOC_REQBUFS, &req) < 0) {
		camera_err(" VIDIOC_REQBUFS failed\n");//分配内存
		close(camera.videofd);
		return -1;
	}

	/* 8.Query the status of buffers and mmap buffers to user space */
	camera.buf_count = req.count;
	camera_dbg(" reqbuf number is %d\n",camera.buf_count);
	camera.buffers = (buffer*)calloc(req.count, sizeof(struct buffer));

	for (n_buffers = 0; n_buffers < (int)req.count; ++n_buffers)
	{
		memset(&buf, 0, sizeof(struct v4l2_buffer));
		if(camera.driver_type == V4L2_CAP_VIDEO_CAPTURE_MPLANE) {
			buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
		} else {
			buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		}

		buf.memory = V4L2_MEMORY_MMAP;
		buf.index = n_buffers;

		if(camera.driver_type == V4L2_CAP_VIDEO_CAPTURE_MPLANE) {
			buf.length = camera.nplanes;
			buf.m.planes =  (struct v4l2_plane *)calloc(buf.length, sizeof(struct v4l2_plane));
		}

		//通过VIDIOC_QUERYBUF获取内核空间的缓冲信息
		if (ioctl(camera.videofd, VIDIOC_QUERYBUF, &buf) == -1) {
			camera_err(" VIDIOC_QUERYBUF error\n");		
			if(camera.driver_type == V4L2_CAP_VIDEO_CAPTURE_MPLANE) {
				free(buf.m.planes);
			}

			free(camera.buffers);
			close(camera.videofd);
			return -1;
		}

		if (camera.driver_type == V4L2_CAP_VIDEO_CAPTURE_MPLANE)
		{
			for(i = 0; i < camera.nplanes; i++) {
				camera.buffers[n_buffers].length[i] = buf.m.planes[i].length;
				camera.buffers[n_buffers].start[i] = mmap(NULL , buf.m.planes[i].length,
						PROT_READ | PROT_WRITE,
						MAP_SHARED , camera.videofd,
						buf.m.planes[i].m.mem_offset);

				camera_dbg(" map buffer index: %d, mem: %p, len: %x, offset: %x\n",
						n_buffers, camera.buffers[n_buffers].start[i],
						buf.m.planes[i].length,
						buf.m.planes[i].m.mem_offset);
			}
			free(buf.m.planes);
		} else {
			camera.buffers[n_buffers].length[0] = buf.length;
			camera.buffers[n_buffers].start[0] = mmap(NULL , buf.length,
					PROT_READ | PROT_WRITE,
					MAP_SHARED , camera.videofd,
					buf.m.offset);
			camera_dbg(" map buffer index: %d, mem: %p, len: %x, offset: %x\n",
					n_buffers, camera.buffers[n_buffers].start[0],
					buf.length,buf.m.offset);
		}
	}

	/* 9.Queue buffers */
	for (n_buffers = 0; n_buffers < (int)req.count; n_buffers++)
	{
		memset(&buf, 0, sizeof(struct v4l2_buffer));
		if(camera.driver_type == V4L2_CAP_VIDEO_CAPTURE_MPLANE) {
			buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
		} else {
			buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		}

		buf.memory= V4L2_MEMORY_MMAP;
		buf.index= n_buffers;

		if (camera.driver_type == V4L2_CAP_VIDEO_CAPTURE_MPLANE) {
			buf.length = camera.nplanes;
			buf.m.planes =  (struct v4l2_plane *)calloc(buf.length, sizeof(struct v4l2_plane));
		}

		if (ioctl(camera.videofd, VIDIOC_QBUF, &buf) == -1) {
			camera_err(" VIDIOC_QBUF error\n");
			if(camera.driver_type == V4L2_CAP_VIDEO_CAPTURE_MPLANE) {
				free(buf.m.planes);
			}

			free(camera.buffers);
			close(camera.videofd);
			return -1;
		}

		if (camera.driver_type == V4L2_CAP_VIDEO_CAPTURE_MPLANE) {
			free(buf.m.planes);
		}
	}

        char *cPtrpmdPath = (char*)"/userdata/camera/";
        char *i2c_name = (char*)"/dev/i2c-2";

	DepthMapWrapperSetUp(i2c_name, cPtrpmdPath);

        if (DepthMapWrapperSetUseCase(usecase[2]) == true)
        {
            camera_print("\nDepthMapWrapperSetUseCase 2 successed! \n");	//2020-3-21
        }

        if(DepthMapWrapperFilterLevel() == true)
        {
     	    camera_print("\nset FilterLevel succeed \n");
        }
        if (DepthMapWrapperSetAutoExposureEnabled(true))
        {
            camera_print("\nset Auto-Expo succeed \n");
        }

        WrapperDepthCamLensParameters camera_parameters;
        DepthMapWrapperGetLensParameters(&camera_parameters);
        printf("cx is %f\n", camera_parameters.principalPoint[0]);
        printf("cy is %f\n", camera_parameters.principalPoint[1]);
        printf("fx is %f\n", camera_parameters.focalLength[0]);
        printf("fy is %f\n", camera_parameters.focalLength[1]);
        printf("p1 is %f\n", camera_parameters.distortionTangential[0]);
        printf("p2 is %f\n", camera_parameters.distortionTangential[1]);
        printf("k1 is %f\n", camera_parameters.distortionRadial[0]);
        printf("k2 is %f\n", camera_parameters.distortionRadial[1]);
        printf("k3 is %f\n", camera_parameters.distortionRadial[2]);

	/* 10.Capture photos */
	if (capture_photo(&camera) < 0) {
		camera_err(" capture_photo return error\n");
		return -1;
	}
	return 0;
}
