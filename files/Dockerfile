FROM ros:humble

#Não entendi o que faz, dps pesquiso pra enteder*
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends apt-utils

#Atualiza pacotes e instala ferramentas
RUN apt-get update && apt-get upgrade -y && \
  apt-get install -y --no-install-recommends \
  locales \
  curl \
  vim \
  git \
  zstd

#Install ZED SDK
COPY ZED_SDK_Ubuntu22_cuda12.8_tensorrt10.9_v5.0.5.zstd.run /root 
RUN chmod +x /root/ZED_SDK_Ubuntu22_cuda12.8_tensorrt10.9_v5.0.5.zstd.run && ./root/ZED_SDK_Ubuntu22_cuda12.8_tensorrt10.9_v5.0.5.zstd.run -- silent

# Instala dependências do sistema
RUN apt-get update && apt-get install -y \
    git \
    curl \
    wget \
    lsb-release \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    python3-vcstool \
    ros-humble-ament-cmake-auto \
 && rm -rf /var/lib/apt/lists/*

# Variável para controle do cache
ARG CACHE_BREAKER=0

RUN mkdir -p /root/ros2_ws/src

RUN cd /root/ros2_ws/src && git clone https://github.com/stereolabs/zed-ros2-wrapper.git && git clone --branch humble --single-branch https://github.com/f1tenth/vesc.git && git clone --recursive https://github.com/UrgNetworks/urg_node2.git && git clone  --branch humble --single-branch https://github.com/ros-drivers/transport_drivers.git

RUN cd /root/ros2_ws && apt-get update && rosdep install --from-paths src --ignore-src -r -y && . /opt/ros/$ROS_DISTRO/setup.sh && colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release

RUN apt-get install tmux -y
