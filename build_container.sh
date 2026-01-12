#/bin/bash
#允许‌任何客户端‌连接到当前用户的 X 服务器（图形显示服务）
xhost +
echo "检查镜像是否存在。"

cd bolight_alg_base
./build_image.sh

cd ..

docker-compose up -d