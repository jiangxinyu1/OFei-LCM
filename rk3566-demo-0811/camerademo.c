#include "camerademo.h"
#include "DepthMapWrapper.h"
#include "time.h"
#include "calibration.h"
#include "unistd.h"

// 两种模式2选1
// #define WHILE_1_MODE
#define NORMAL_MODE

extern uint8_t raw12_img[336*1557];
extern char ir_image[224*114];
extern char depth_image[224*114*2];
extern char pcloud_image[224*114*sizeof(pc_pkt_t)];
extern char depth_data[224*114*sizeof(depth_data_pkt_t)];
extern unsigned int pt_count;


int capture_photo(struct camera_hal *camera)
{
	enum v4l2_buf_type type;
	struct timeval tv;
	struct v4l2_buffer buf;
	int i = 0, j = 0, ret = 0, np = 0;
	char source_data_path[100];
	fd_set fds;

	struct timespec time1 = {0,0};
	struct timespec time2 = {0,0};
	float temp;

	bool rc=false;

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

#ifdef NORMAL_MODE
	while (np < camera->photo_num)
#endif

#ifdef WHILE_1_MODE
	while (1)
#endif

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
		memset(source_data_path, 0, sizeof(source_data_path));
		sprintf(source_data_path, "%s/raw_%d.raw", camera->save_path,  np+1);

#ifdef NORMAL_MODE
		// save_frame_to_file(source_data_path, camera->buffers[buf.index].start[0], camera->buffers[buf.index].length[0]-224*2);
#endif
        /* Decode Raw Data */
		//clock_gettime(CLOCK_REALTIME, &time1);	
		DepthMapWrapperProcessFrame((char*)(camera->buffers[buf.index].start[0]), ir_image, depth_image, pcloud_image, depth_data, &pt_count);
		//clock_gettime(CLOCK_REALTIME, &time2);	
		//temp = (time2.tv_nsec - time1.tv_nsec) / 1000000;
		//printf("\n	time = %f ms		\n",temp);

                DepthMapWrapperGetExposureTime();

		/* Save others */

		// sprintf(source_data_path, "%s/%s_%d%s", camera->save_path, "IR", np+1, ".raw");
		// camera_print(" get pclout %d , save IR img to %s\n", pt_count, source_data_path);
        
#ifdef NORMAL_MODE
        // save_frame_to_file(source_data_path, ir_image, 224*114);
#endif
		

		sprintf(source_data_path, "%s/%s_%d%s", camera->save_path, "Depth", np+1, ".raw");

#ifdef NORMAL_MODE
		// save_frame_to_file(source_data_path, depth_image, 224*114*2);
#endif
		sprintf(source_data_path, "%s/%s_%d%s", camera->save_path, "PointCloud", np+1, ".ply");

#ifdef NORMAL_MODE
		writePointCloud(source_data_path, pcloud_image, pt_count);
#endif
		//printf(source_data_path, "%s/%s_%d%s", camera->save_path, "DepthData", np+1, ".ply");
		//writeDepthData(source_data_path, depth_data, pt_count);

		/* Queue buf */
		if (ioctl(camera->videofd, VIDIOC_QBUF, &buf) == 0) {
			camera_dbg("************QBUF[%d] FINISH**************\n",buf.index);
		} else {
			camera_err("*****QBUF FAIL*****\n");
			ret = -1;
			break;
		}

		np++;
    sleep(300);
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

