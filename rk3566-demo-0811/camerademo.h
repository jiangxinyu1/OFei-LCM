/*
 * @Author: your name
 * @Date: 2021-08-12 21:39:24
 * @LastEditTime: 2021-08-13 15:29:49
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /rk3566-demo-0811/camerademo.h
 */
#ifndef CAMERADEMO__H__
#define CAMERADEMO__H__

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

#include "common.h"

extern int camera_dbg_en;

#define get_format_name(fmt) \
	(fmt == V4L2_PIX_FMT_SBGGR12)?"SBGGR12":"Unknown"

#define DEFAULT_I2C_BUS  "/dev/i2c-2"
#define PMD_SPC_FOLDER   "/userdata/camera/pmd.spc"




//for internel debug
#define camera_dbg(x,arg...) do{ \
                                if(camera_dbg_en) \
                                    printf(" [CAMERA_DEBUG] " x ,##arg); \
                            }while(0)

//print when error happens
#define camera_err(x,arg...) do{ \
                                printf(" \033[1m\033[;31m [CAMERA_ERR] " x ,##arg); \
                                printf(" \033[0m "); \
                                fflush(stdout); \
                            }while(0)

#define camera_prompt(x,arg...) do{ \
                                printf(" \033[1m\033[;32m[CAMERA_PROMPT] " x "\033[0m",##arg); \
                                fflush(stdout); \
                            }while(0)

#define camera_warn(x,arg...) printf("[CAMERA_WARN]" x ,##arg)
//print unconditional, for important info
#define camera_print(x,arg...) printf("[CAMERA]" x ,##arg)

struct buffer
{
    void *start[3];
    size_t length[3];
};

typedef struct camera_hal
{
    int camera_index;
    int videofd;
    int driver_type;
    int photo_num;
    char save_path[64];
    struct buffer *buffers;
    int buf_count;
    int nplanes;
    unsigned int win_width;
    unsigned int win_height;
    unsigned int pixelformat;
}camera_handle;

int capture_photo(struct camera_hal *camera);

#endif
