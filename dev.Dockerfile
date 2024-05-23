##############################################################################
##                           1. stage: Base Image                           ##
##############################################################################
ARG ROS_DISTRO=humble
FROM osrf/ros:$ROS_DISTRO-desktop as base

# Configure DDS
COPY dds_profile.xml /opt/misc/dds_profile.xml
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/opt/misc/dds_profile.xml

# Create user
ARG USER=logilab
ARG PASSWORD=automaton
ARG UID=1000
ARG GID=1000
ENV USER=$USER
RUN groupadd -g $GID $USER \
    && useradd -m -u $UID -g $GID -p "$(openssl passwd -1 $PASSWORD)" \
    --shell $(which bash) $USER -G sudo

# Setup workpace
USER $USER
RUN mkdir -p /home/$USER/ros2_ws/src
WORKDIR /home/$USER/ros2_ws

##############################################################################
##                     2. stage: set up description pkg                     ##
##############################################################################
FROM base as sew_description

USER root
RUN apt-get update && apt-get install -y ros-${ROS_DISTRO}-xacro
RUN apt-get update && apt-get install -y ros-${ROS_DISTRO}-joint-state-publisher-gui
USER $USER


##############################################################################
##                           3. stage: set up gazebo                        ##
##############################################################################
FROM sew_description as gazebo_testenviroment

USER root
RUN apt-get update && apt-get install -y ros-${ROS_DISTRO}-teleop-twist-joy \
    ros-${ROS_DISTRO}-teleop-twist-keyboard \
    ros-${ROS_DISTRO}-joy \
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs

RUN apt-get update && apt-get install -y joystick
USER $USER

##############################################################################
##                         4. stage: set up nav2 stack                      ##
##############################################################################
FROM gazebo_testenviroment as sew_navigation

USER root
RUN apt-get update && apt-get install -y ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-* \
    ros-${ROS_DISTRO}-slam-toolbox \
    ros-${ROS_DISTRO}-twist-mux

USER $USER

##############################################################################
##                      5. stage: set up igus packages                      ##
##############################################################################
FROM sew_navigation as sew_mobile_manipulator

#install irc packages from github?
#needed ros2 control packages?

#CMD [/bin/bash]
