# syntax=docker/dockerfile:1.0.0-experimental
###pull###
FROM osrf/ros:noetic-desktop-full
ADD ./config/.bashrc /root/.bashrc
ADD ./launch/brushee_object_detector.launch /root/brushee_object_detector.launch

SHELL ["/bin/bash", "-c"]
ENV HOME /root
WORKDIR $HOME
RUN apt update && \
    apt install -y curl python3-osrf-pycommon python3-rosdep python3-catkin-tools \
                   python3-rosdep python3-rosinstall python3-rosinstall-generator \
                   python3-wstool build-essential git openssh-server tmux \
                   ros-noetic-libuvc-camera

### setup ssh###
RUN mkdir -m 700 $HOME/.ssh
RUN ssh-keyscan github.com > $HOME/.ssh/known_hosts

### ROS setup###
WORKDIR $HOME
RUN mkdir -p $HOME/catkin_ws/src
WORKDIR $HOME/catkin_ws/src
RUN --mount=type=ssh git clone --recursive git@github.com:YasunoriHirakawa/theta_s_ros.git && \
                     git clone https://github.com/Hori3538/depth_estimator.git && \
                     git clone -b dev https://github.com/Hori3538/camera_apps.git && \
                     git clone -b dev https://github.com/Hori3538/camera_apps_msgs.git && \
    rosdep update && rosdep install -i -r -y --from-paths .

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
