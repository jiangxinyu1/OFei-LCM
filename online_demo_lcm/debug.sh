#! /bin/bash
###
 # @Author: your name
 # @Date: 2021-05-31 18:22:49
 # @LastEditTime: 2021-08-27 17:14:09
 # @LastEditors: Please set LastEditors
 # @Description: In User Settings Edit
 # @FilePath: /vscodeTest/debug.sh
### 

echo "Configuring and building project .... "
if [ ! -d "Debug" ];then
 mkdir Debug
fi
cd Debug
cmake -DCMAKE_BUILD_TYPE=Debug .. -DCMAKE_TOOLCHAIN_FILE=~/.Toolchain/arm_toolchain.cmake
make -j8
date