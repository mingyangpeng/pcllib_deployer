#!/bin/bash

# 定义镜像名称和标签
IMAGE_NAME="ros2_alg_base" 
IMAGE_TAG="v0.0.1"
 

DockerID=$(docker images -q $IMAGE_NAME:$IMAGE_TAG)

echo "Query Image ID:" $DockerID

IDcount=${#DockerID}

# 使用docker images命令查找镜像，并检查返回结果
if [ $IDcount -gt 0 ]; then
    echo "镜像 $IMAGE_NAME:$IMAGE_TAG 存在。"
else
    echo "镜像 $IMAGE_NAME:$IMAGE_TAG 不存在,准备构建镜像。"
    docker build --no-cache -t $IMAGE_NAME:$IMAGE_TAG .
    echo "镜像 $IMAGE_NAME:$IMAGE_TAG 构建完成。"
fi
