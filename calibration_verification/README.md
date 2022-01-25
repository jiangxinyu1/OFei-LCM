<!--
 * @Author: jiangxinyu
 * @Date: 2021-08-20 16:02:33
 * @LastEditTime: 2021-09-06 09:49:43
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /calibration_verification/README.md
-->

# 0  gflags参数文件

工程目录下的 param1.cmd 和 param2.cmd 分别为两个机器的内参与外参

```shell
--flagfile=../param1.cmd
```

# 1 简易标定->外参验证工程（使用Marker）
## 算法流程

像素平面提取Marker黑色平面的上边缘（Canny和Sobel算子处理）

根据的IR图和depth图的信息进行区域增长，提取出Marker像素平面的平面点

利用内参和外参将图像坐标系的点投影到机器人坐标系下的三维点云

Ransca进一步拟合点云平面，计算平面方程（降噪）

通过PCA重新的拟合内点平面，计算平面方程

计算点到平面的距离（带方向，所有点）

Ransca使用投影出来的直线点，拟合直线

通过PCA计算直线点主方向

直线主方向和惯性方向计算内积

计算直线点的高度的均值和方差


# 2 IR图-直方图均衡化


# 3 点云的转换工程（Camera系->Robot系）

点云数据通过外参转换而来，并根据其距离地面的高度分层显示



# 4 在IR图对应位置显示异常点云数据



# 5 读取ply点云文件并显示
