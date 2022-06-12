# syntax=docker/dockerfile:1.0.0-experimental
###pull###
#FROM osrf/ros:noetic-desktop-full
FROM scratch
ADD ubuntu-focal-oci-arm64-root.tar.gz /

ADD ./config/.bashrc /root/.bashrc
ADD ./launch/brushee_object_detector.launch /root/brushee_object_detector.launch


SHELL ["/bin/bash", "-c"]
ENV HOME /root
WORKDIR $HOME
ENV TZ Asia/Tokyo
ENV DEBIAN_FRONTEND=noninteractive
RUN apt update && \
    apt install -y lsb-release && \
    sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt install tzdata && \
    ln -snf /usr/share/zoneinfo/Asia/Tokyo /etc/localtime && \
    apt install -y curl gnupg && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    apt update -y && \
    apt install -y -o Dpkg::Options::="--force-confdef" -o Dpkg::Options::="--force-confold" \
                   ros-noetic-ros-base python3-osrf-pycommon python3-rosdep python3-catkin-tools \
                   python3-rosdep python3-rosinstall python3-rosinstall-generator \
                   python3-wstool build-essential git openssh-server tmux \
                   ros-noetic-libuvc-camera ros-noetic-cv-bridge \
                   ros-noetic-pcl-ros ros-noetic-pcl-msgs ros-noetic-eigen-conversions

## install cuda ###
# RUN wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/sbsa/cuda-ubuntu2004.pin && \
#     mv cuda-ubuntu2004.pin /etc/apt/preferences.d/cuda-repository-pin-600 && \
#     wget https://developer.download.nvidia.com/compute/cuda/11.3.0/local_installers/cuda-repo-ubuntu2004-11-3-local_11.3.0-465.19.01-1_arm64.deb && \
#     dpkg -i cuda-repo-ubuntu2004-11-3-local_11.3.0-465.19.01-1_arm64.deb && \
#     apt-key add /var/cuda-repo-ubuntu2004-11-3-local/7fa2af80.pub && \
#     apt update && \
#     apt install -y -o Dpkg::Options::="--force-confdef" -o Dpkg::Options::="--force-confold" \
#                  cuda-11-3

RUN wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/sbsa/cuda-ubuntu2004.pin && \
    mv cuda-ubuntu2004.pin /etc/apt/preferences.d/cuda-repository-pin-600 && \
    wget https://developer.download.nvidia.com/compute/cuda/11.6.0/local_installers/cuda-repo-ubuntu2004-11-6-local_11.6.0-510.39.01-1_arm64.deb && \
    dpkg -i cuda-repo-ubuntu2004-11-6-local_11.6.0-510.39.01-1_arm64.deb && \
    apt-key add /var/cuda-repo-ubuntu2004-11-6-local/7fa2af80.pub && \
    apt update && \
    apt install -y -o Dpkg::Options::="--force-confdef" -o Dpkg::Options::="--force-confold" \
                   cuda-11-6

### install cudnn ###
WORKDIR /root
COPY ./cudnn-local-repo-ubuntu2004-8.4.0.27_1.0-1_arm64.deb /root/cudnn-local-repo-ubuntu2004-8.4.0.27_1.0-1_arm64.deb
RUN dpkg -i /root/cudnn-local-repo-ubuntu2004-8.4.0.27_1.0-1_arm64.deb && \
    cp -r /var/cudnn-local-repo-ubuntu2004-8.4.0.27/Release.gpg /usr/share/keyrings/ && \
    apt update && \
    apt install -y -o Dpkg::Options::="--force-confdef" -o Dpkg::Options::="--force-confold" \
    libcudnn8=8.4.0.27-1+cuda11.6 libcudnn8-dev=8.4.0.27-1+cuda11.6 libcudnn8-samples=8.4.0.27-1+cuda11.6

### build opencv ###
WORKDIR /root
RUN apt update && apt upgrade -y &&\
    apt install -y -o Dpkg::Options::="--force-confdef" -o Dpkg::Options::="--force-confold" \
                build-essential cmake unzip pkg-config libjpeg-dev libpng-dev libtiff-dev \
                libavcodec-dev libavformat-dev libswscale-dev libv4l-dev libxvidcore-dev libx264-dev \
                libgtk-3-dev libblas-dev liblapack-dev gfortran python3-dev &&\
    wget -O opencv-4.5.5.zip https://github.com/opencv/opencv/archive/4.5.5.zip && \
    unzip -q opencv-4.5.5.zip && \
    mv opencv-4.5.5 opencv && \
    rm -f opencv-4.5.5.zip

RUN wget -O opencv_contrib-4.5.5.zip https://github.com/opencv/opencv_contrib/archive/4.5.5.zip && \
    unzip -q opencv_contrib-4.5.5.zip && \
    mv opencv_contrib-4.5.5 opencv_contrib && \
    rm -f opencv_contrib-4.5.5.zip


### build opencv4.5.2 ###
RUN apt update && apt upgrade -y &&\
    apt install -y -o Dpkg::Options::="--force-confdef" -o Dpkg::Options::="--force-confold" \
                build-essential cmake pkg-config unzip yasm git checkinstall \
                libjpeg-dev libpng-dev libtiff-dev \
                libavcodec-dev libavformat-dev libswscale-dev libavresample-dev \
                libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
                libxvidcore-dev x264 libx264-dev libfaac-dev libmp3lame-dev libtheora-dev \
                libfaac-dev libmp3lame-dev libvorbis-dev \
                libopencore-amrnb-dev libopencore-amrwb-dev \
                libdc1394-22 libdc1394-22-dev libxine2-dev libv4l-dev v4l-utils
WORKDIR /usr/include/linux
RUN ln -s -f ../libv4l1-videodev.h videodev.h
WORKDIR /root
RUN apt install -y -o Dpkg::Options::="--force-confdef" -o Dpkg::Options::="--force-confold" \
                libgtk-3-dev python3-dev python3-pip
RUN pip3 install  pip numpy &&\
    apt install -y -o Dpkg::Options::="--force-confdef" -o Dpkg::Options::="--force-confold" \
                python3-testresources libtbb-dev libprotobuf-dev protobuf-compiler \
                libgoogle-glog-dev libgflags-dev libgphoto2-dev libeigen3-dev libhdf5-dev doxygen &&\
    mkdir /root/Downloads
WORKDIR /root/Downloads
RUN wget -O opencv.zip https://github.com/opencv/opencv/archive/refs/tags/4.5.2.zip && \
    wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/refs/tags/4.5.2.zip && \
    unzip opencv.zip && \
    unzip opencv_contrib.zip

RUN pip install virtualenv virtualenvwrapper &&\
    rm -rf ~/.cache/pip &&\
    echo "Edit ~/.bashrc" &&\
    export WORKON_HOME=$HOME/.virtualenvs &&\
    export VIRTUALENVWRAPPER_PYTHON=/usr/bin/python3 &&\
    source /usr/local/bin/virtualenvwrapper.sh &&\
    mkvirtualenv cv -p python3 &&\
    pip install numpy &&\
    cd /root/opencv/ && \
    mkdir build && \
    cd build && \
    cmake -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D WITH_TBB=ON \
    -D ENABLE_FAST_MATH=1 \
    -D CUDA_FAST_MATH=1 \
    -D WITH_CUBLAS=1 \
    -D WITH_CUDA=ON \
    -D BUILD_opencv_cudacodec=OFF \
    -D WITH_CUDNN=ON \
    -D OPENCV_DNN_CUDA=ON \
    -D CUDA_ARCH_BIN=7.2 \
    -D CUDA_ARCH_PTX=7.2 \
    -D WITH_V4L=ON \
    -D WITH_QT=OFF \
    -D WITH_OPENGL=ON \
    -D WITH_GSTREAMER=ON \
    -D OPENCV_GENERATE_PKGCONFIG=ON \
    -D OPENCV_PC_FILE_NAME=opencv.pc \
    -D OPENCV_ENABLE_NONFREE=ON \
    -D OPENCV_PYTHON3_INSTALL_PATH=~/.virtualenvs/cv/lib/python3.8/site-packages \
    -D PYTHON_EXECUTABLE=/root/.virtualenvs/cv/bin/python \
    -D OPENCV_EXTRA_MODULES_PATH=/root/opencv_contrib/modules \
    -D INSTALL_PYTHON_EXAMPLES=OFF \
    -D INSTALL_C_EXAMPLES=OFF \
    -D BUILD_EXAMPLES=OFF .. && \
    make -j$(nproc) && \
    make install && \
    /bin/bash -c 'echo "/usr/local/lib" >> /etc/ld.so.conf.d/opencv.conf' && \
    ldconfig




### setup ssh###
RUN mkdir -m 700 $HOME/.ssh
RUN ssh-keyscan github.com > $HOME/.ssh/known_hosts

### ROS setup###
WORKDIR $HOME
RUN mkdir -p $HOME/catkin_ws/src
WORKDIR $HOME/catkin_ws/src
RUN apt install -y -o Dpkg::Options::="--force-confdef" -o Dpkg::Options::="--force-confold" \
                     ros-noetic-tf2-geometry-msgs
RUN --mount=type=ssh git clone --recursive git@github.com:YasunoriHirakawa/theta_s_ros.git && \
                     git clone https://github.com/Hori3538/depth_estimator.git && \
                     git clone -b dev https://github.com/Hori3538/camera_apps.git && \
                     git clone -b dev https://github.com/Hori3538/camera_apps_msgs.git && \
                     rosdep init && rosdep update && rosdep install -i -r -y --from-paths .

##catkin build##
WORKDIR $HOME/catkin_ws
RUN source /opt/ros/noetic/setup.bash && catkin build -DMAKE_BUILD_TYPE=Release
RUN source $HOME/catkin_ws/devel/setup.bash

# ENV ROS_PACKAGE_PATH $HOME/catkin_ws/src:$ROS_PACKAGE_PATH
# ENV ROS_WORKSPACE $HOME/catkin_ws
### starting humanditection###
# CMD source $HOME/.bashrc && \
#     source /opt/ros/noetic/setup.bash && \
#     source $HOME/catkin_ws/devel/setup.bash && \
#     roslaunch theta_s_ros theta_s_ros.launch
# CMD source $HOME/.bashrc && roslaunch $HOME/brushee_object_detector.launch
# ENTRYPOINT ["/root/ros_enttrypoint.sh"]
CMD ["bash"]
