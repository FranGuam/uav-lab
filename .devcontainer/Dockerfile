FROM osrf/ros:noetic-desktop-full

# Update all packages
RUN apt update && apt upgrade -y

# Install Git
RUN apt install -y git

# Create Symlinks for Python3
RUN ln -s /usr/bin/python3 /usr/bin/python

# Install Python3-pip
RUN apt install -y python3-pip

# Configure pip source for Mainland China users
# RUN pip config set global.index-url https://mirrors.tuna.tsinghua.edu.cn/pypi/web/simple

# # Rosdep (for non-Mainland China users)
# RUN rosdep init
# RUN rosdep update

# Rosdep (for Mainland China users)
# 手动模拟 rosdep init
RUN apt install -y curl
RUN mkdir -p /etc/ros/rosdep/sources.list.d/
RUN curl -o /etc/ros/rosdep/sources.list.d/20-default.list https://mirrors.tuna.tsinghua.edu.cn/github-raw/ros/rosdistro/master/rosdep/sources.list.d/20-default.list
# 为 rosdep update 换源
RUN export ROSDISTRO_INDEX_URL=https://mirrors.tuna.tsinghua.edu.cn/rosdistro/index-v4.yaml
# 为了持久化该设定，可以将其写入 .bashrc 中
RUN echo 'export ROSDISTRO_INDEX_URL=https://mirrors.tuna.tsinghua.edu.cn/rosdistro/index-v4.yaml' >> ~/.bashrc
RUN rosdep update

# Source the ROS setup file
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
