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
    ros-${ROS_DISTRO}-joy* \
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-gazebo-ros2-control

RUN apt-get update && apt-get install -y joystick
RUN apt-get update && apt-get install -y xterm
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

# install necessary packages for ros2 control
USER root
RUN apt-get update && apt-get install -y ros-${ROS_DISTRO}-controller-interface \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-hardware-interface \
    ros-${ROS_DISTRO}-pluginlib \
    ros-${ROS_DISTRO}-rclcpp \
    ros-${ROS_DISTRO}-rclcpp-lifecycle \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-rosidl-default-generators \
    ros-${ROS_DISTRO}-std-srvs
USER $USER


# install necessary packages for moveit
USER root
RUN DEBIAN_FRONTEND=noninteractive \
	apt update && apt install -y  \
    ros-${ROS_DISTRO}-moveit  \
    ros-${ROS_DISTRO}-moveit-common  \
    ros-${ROS_DISTRO}-moveit-servo  \
    ros-${ROS_DISTRO}-moveit-ros-perception  \
    ros-${ROS_DISTRO}-joint-trajectory-controller  \
    ros-${ROS_DISTRO}-joint-state-broadcaster  \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-sensor-msgs-py  \
    ros-${ROS_DISTRO}-rqt-controller-manager
USER $USER


#install dependencies to calculate with affine transformations
USER root
RUN DEBIAN_FRONTEND=noninteractive \
    apt update && apt install -y  \
    ros-${ROS_DISTRO}-tf-transformations

RUN apt-get update && apt-get install -y pip
RUN apt-get update && apt-get install -y libnlopt*

RUN pip install transforms3d
RUN pip install scipy

USER $USER



# Build the workspace 
# RUN cd /home/$USER/ros2_ws/ && \
#     . /opt/ros/$ROS_DISTRO/setup.sh && \
#     colcon build --packages-select irc_ros_msgs && \
#     colcon build --packages-select irc_ros_description && \
#     colcon build --packages-select irc_ros_moveit_config && \
#     colcon build --packages-select irc_ros_hardware && \
#     colcon build --packages-select irc_ros_controllers && \
#     colcon build --packages-select irc_ros_bringup

#CMD [/bin/bash]
