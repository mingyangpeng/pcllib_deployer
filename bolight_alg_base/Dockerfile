FROM registry.cn-hangzhou.aliyuncs.com/bolight/ubuntu24_jazzy:v0.0.1

# 设置环境变量，避免交互式配置
ENV DEBIAN_FRONTEND=noninteractive

ENV TZ=Asia/Shanghai
 
# 更新软件源 
RUN rm -f /etc/apt/sources.list.d/ros2-latest.list || true
RUN echo "deb [trusted=yes] http://mirrors.aliyun.com/ros2/ubuntu/ $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-aliyun.list

RUN apt update && apt install -y \
    tzdata\
    ntp\
    net-tools \
    vim \
    openssh-server \
    sudo \
    build-essential \
    graphviz \
    graphviz-dev \
    libpcl-dev \
    python3-colcon-common-extensions \
    python3-argcomplete \
    python3-vcstool \
    python3-venv \
    python3-pip \
    ccache \
&& apt clean \
&& rm -rf /var/lib/apt/lists/*

WORKDIR /root/code
# 在虚拟环境中安装open3d

# 创建虚拟环境bolight 并安装open3d
RUN python3 -m venv /root/bolight
ENV VIRTUAL_ENV=/root/bolight
ENV PATH="$VIRTUAL_ENV/bin:$PATH" 
  
RUN pip install -i https://mirrors.aliyun.com/pypi/simple catkin-pkg empy lark numpy opencv-python open3d

# ssh服务配置
RUN mkdir /var/run/sshd
ARG USERNAME=root
RUN echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
 
#USER $USERNAME
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
RUN echo "source /root/bolight/bin/activate" >> ~/.bashrc
RUN echo "export ROS_LOCALHOST_ONLY=1" >> ~/.bashrc 
RUN echo "export ROS_DOMAIN_ID=88">> ~/.bashrc
RUN echo "source ~/code/install/setup.bash">> ~/.bashrc




