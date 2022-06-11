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

### install cuda ###
RUN wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/sbsa/cuda-ubuntu2004.pin && \
    mv cuda-ubuntu2004.pin /etc/apt/preferences.d/cuda-repository-pin-600 && \
    wget https://developer.download.nvidia.com/compute/cuda/11.3.0/local_installers/cuda-repo-ubuntu2004-11-3-local_11.3.0-465.19.01-1_arm64.deb && \
    dpkg -i cuda-repo-ubuntu2004-11-3-local_11.3.0-465.19.01-1_arm64.deb && \
    apt-key add /var/cuda-repo-ubuntu2004-11-3-local/7fa2af80.pub && \
    apt update && \
    apt install -y -o Dpkg::Options::="--force-confdef" -o Dpkg::Options::="--force-confold" \
                 cuda-11-3



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

ENV ROS_PACKAGE_PATH $HOME/catkin_ws/src:$ROS_PACKAGE_PATH
ENV ROS_WORKSPACE $HOME/catkin_ws
### starting humanditection###
# CMD source $HOME/.bashrc && \
#     source /opt/ros/noetic/setup.bash && \
#     source $HOME/catkin_ws/devel/setup.bash && \
#     roslaunch theta_s_ros theta_s_ros.launch
# CMD source $HOME/.bashrc && roslaunch $HOME/brushee_object_detector.launch
# ENTRYPOINT ["/root/ros_enttrypoint.sh"]
CMD ["bash"]
