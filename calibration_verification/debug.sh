#! /bin/bash
###
 # @Author: your name
 # @Date: 2021-05-31 18:22:49
 # @LastEditTime: 2021-05-31 21:22:01
 # @LastEditors: your name
 # @Description: In User Settings Edit
 # @FilePath: /vscodeTest/debug.sh
### 

echo "Configuring and building project .... "
if [ ! -d "Debug" ];then
 mkdir Debug
fi
cd Debug
cmake -DCMAKE_BUILD_TYPE=Debug .. 
make -j8
date