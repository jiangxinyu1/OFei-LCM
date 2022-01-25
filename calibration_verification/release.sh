###
 # @Author: your name
 # @Date: 2021-05-31 18:21:33
 # @LastEditTime: 2021-05-31 18:21:51
 # @LastEditors: Please set LastEditors
 # @Description: In User Settings Edit
 # @FilePath: /vscodeTest/release.sh
### 
#! /bin/bash

echo "Configuring and building project .... "
if [ ! -d "Release" ];then
 mkdir Release
fi
cd Release
cmake -DCMAKE_BUILD_TYPE=Release .. 
make -j8
date