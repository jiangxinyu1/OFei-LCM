
<!--
 * @Author: jiangxinyu
 * @Date: 2021-08-13 10:42:21
 * @LastEditTime: 2021-11-08 21:16:45
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /rk3566-demo-0811/README.md
-->

# 欧菲模组测试工程说明

使用此测试工程在机器端的进行抓图及/LCM实时发布图像的数据




## 1 配置及操作

### 1 根据具体工程代码，在扫地机/data/目录下新建 camera 目录，拷贝动态库，标定文件，可执行程序到扫地机 /data/camera/ 目录。

动态库和标定文件存放在  ./third/
编译出的可执行程序存放在  ./bin/

进入相应目录

```shell
scp ./* root@ip:/data/camera/
```

### 2 到机器端 /data/camera/ ，配置ISP（必须配置），执行程序。

```
media-ctl -d /dev/media1 --set-v4l2 '"rkisp-isp-subdev":0[crop:(0,0)/224x1035]'
media-ctl -d /dev/media1 --set-v4l2 '"rkisp-isp-subdev":0[fmt:SBGGR12_1X12/224x1035]'
media-ctl -d /dev/media1 --set-v4l2 '"rkisp-isp-subdev":2[crop:(0,0)/224x1035]'
media-ctl -d /dev/media1 --set-v4l2 '"rkisp-isp-subdev":2[fmt:SBGGR12_1X12/224x1035]'
```

编译时会编译出两个可执行程序:

1. depthmap_demo -> 欧菲提供的原始抓图程序（连续五张图片）
2. depthmap_demo_LCM -> 将原始数据转成的LCM消息发布用于测试的程序

#### 2.1 抓取连续帧图像文件（depth ir 点云）

```shell
chmod 777 depthmap_demo
LD_LIBRARY_PATH=/data/camera ./depthmap_demo
```

图像及点云文件的都会存储在扫地机 /data/camera/ 目录下，建议在PC端使用 adb pull 直接拉取。

#### 2.2 LCM发布消息（depth ir 点云）

扫地机端：执行程序，发布LCM消息

```shell
chmod 777 depthmap_demo_LCM
LD_LIBRARY_PATH=/data/camera ./depthmap_demo_LCM
```-

PC端：使用lcm2ros工程的将lcm话题转成ros话题发布

```shell
./lcm2ros.sh ip
```

ip：扫地机的ip的最后一个字段。

"pointcloud_3d"的frame_id = "base_link"

rviz节点订阅 "depth_image" "ir_image" "pointcloud_3d" 话题，即可实时看数据。



####  2.3 使用命令抓单张图方法（用于测试驱动）


10帧驱动对应的版本ISP

```
v4l2-ctl -d /dev/video5 --set-selection=target=crop,top=0,left=0,width=224,height=1035 --set-fmt-video=width=224,height=1035,pixelformat=BG12 --stream-mmap=3 --stream-to=/data/cap.raw --stream-count=1 --stream-poll
```

15帧 -> RKCF
```
v4l2-ctl -d /dev/video0 --set-fmt-video=width=224,height=1035,pixelformat=BG12 --set-crop=top=0,left=0,width=224,height=1035 --stream-mmap=3 --stream-count=1 --stream-to=/tmp/cap.raw --stream-poll
```



## 2 欧菲SDK数据及设置

### 2.1 数据存储格式

```c++
char ir_image[224*114];
char depth_image[224*114*2];
char pcloud_image[224*114*sizeof(pc_pkt_t)];

typedef struct pc_pack{
  float X;
  float Y;
  float Z;
  float c;
}pc_pkt_t;
```
Depth图：depth_image ; 224*114；每个数据用16位存储，低13位为深度值，高3位为"置信度"。在转换成LCM消息时，直接做了位与取出深度值。




### 2.2 相机帧率设置

使用如下函数设置

```c++
        if (DepthMapWrapperSetUseCase(usecase[0]) == true)
        {
            camera_print("\nDepthMapWrapperSetUseCase 0 successed! \n");
```

可选 5FPS, 10FPS, 15FPS, 30FPS;




## 3 看图工具

ubuntu上安装 imageJ 工具。

```shell
sudo apt-get install default-jre
sudo apt-get install default-jdk
sudo apt-get install imagej
```



## 4 需要和欧菲确认的问题

1. 原始数据的格式（深度图数据包含什么信息）

2. 相机内参如何获取？
    读取 pmd.spc

3. 图像是否已经经过去畸变处理？




512M media1 video5
```
media-ctl -d /dev/media1 --set-v4l2 '"rkisp-isp-subdev":0[crop:(0,0)/224x1035]'
media-ctl -d /dev/media1 --set-v4l2 '"rkisp-isp-subdev":0[fmt:SBGGR12_1X12/224x1035]'
media-ctl -d /dev/media1 --set-v4l2 '"rkisp-isp-subdev":2[crop:(0,0)/224x1035]'
media-ctl -d /dev/media1 --set-v4l2 '"rkisp-isp-subdev":2[fmt:SBGGR12_1X12/224x1035]'
LD_LIBRARY_PATH=/data/camera ./depthmap_demo 
```

1T 之后改成 media0 video0
```
media-ctl -d /dev/media0 --set-v4l2 '"rkisp-isp-subdev":0[crop:(0,0)/224x1035]'
media-ctl -d /dev/media0 --set-v4l2 '"rkisp-isp-subdev":0[fmt:SBGGR12_1X12/224x1035]'
media-ctl -d /dev/media0 --set-v4l2 '"rkisp-isp-subdev":2[crop:(0,0)/224x1035]'
media-ctl -d /dev/media0 --set-v4l2 '"rkisp-isp-subdev":2[fmt:SBGGR12_1X12/224x1035]'
LD_LIBRARY_PATH=/data/camera ./depthmap_demo 
```




## RGB 抓图命令
```
v4l2-ctl -d /dev/video5 --set-selection=target=crop,top=0,left=0,width=1920,height=1080 --set-fmt-video=width=1920,height=1080,pixelformat=YU12 --stream-mmap=3 --stream-to=/tmp/cif.yuv --stream-count=1  --stream-poll
```


```
ffplay -x 960 -y 540 -video_size 1920x1080 -pixel_format yuv420p cif.yuv
```

