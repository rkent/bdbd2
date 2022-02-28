#!/bin/bash
set -x


# Installation of ros2 on Jetson Ubuntu 18
# Adapted from https://github.com/dusty-nv/jetson-containers/blob/master/Dockerfile.ros.galactic

ROS_PKG=ros_base
ROS_DISTRO=galactic
ROS_ROOT=/opt/ros/${ROS_DISTRO}

DEBIAN_FRONTEND=noninteractive
cd /tmp

# change the locale from POSIX to UTF-8
sudo locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
export PYTHONIOENCODING=utf-8

# 
# add the ROS deb repo to the apt sources list
#
sudo apt-get update && \
    sudo apt-get install -y --no-install-recommends \
		curl \
		wget \
		gnupg2 \
		lsb-release \
		ca-certificates

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 
# install development packages
#
sudo apt-get update && \
    sudo apt-get install -y --no-install-recommends \
		build-essential \
		cmake \
		git \
		libbullet-dev \
		libpython3-dev \
		python3-colcon-common-extensions \
		python3-flake8 \
		python3-pip \
		python3-numpy \
		python3-pytest-cov \
		python3-rosdep \
		python3-setuptools \
		python3-vcstool \
		python3-rosinstall-generator \
		libasio-dev \
		libtinyxml2-dev \
		libcunit1-dev \
		libgazebo9-dev \
		gazebo9 \
		gazebo9-common \
		gazebo9-plugin-base
  
  # install some pip packages needed for testing
sudo python3 -m pip install -U \
		argcomplete \
		flake8-blind-except \
		flake8-builtins \
		flake8-class-newline \
		flake8-comprehensions \
		flake8-deprecated \
		flake8-docstrings \
		flake8-import-order \
		flake8-quotes \
		pytest-repeat \
		pytest-rerunfailures \
		pytest
  
  # 
# install OpenCV (with CUDA)
#
export OPENCV_URL=https://nvidia.box.com/shared/static/5v89u6g5rb62fpz4lh0rz531ajo2t5ef.gz
export OPENCV_DEB=OpenCV-4.5.0-aarch64.tar.gz

sudo apt-get purge -y '*opencv*' || echo "previous OpenCV installation not found" && \
    mkdir opencv && \
    cd opencv && \
    wget --quiet --show-progress --progress=bar:force:noscroll --no-check-certificate ${OPENCV_URL} -O ${OPENCV_DEB} && \
    tar -xzvf ${OPENCV_DEB} && \
    sudo dpkg -i --force-depends *.deb && \
    sudo apt-get update && \
    sudo apt-get install -y -f --no-install-recommends && \
    sudo dpkg -i *.deb && \
    cd ../ && \
    rm -rf opencv && \
    sudo cp -r /usr/include/opencv4 /usr/local/include/opencv4 && \
    sudo cp -r /usr/lib/python3.6/dist-packages/cv2 /usr/local/lib/python3.6/dist-packages/cv2

# 
# upgrade cmake - https://stackoverflow.com/a/56690743
# this is needed to build some of the ROS2 packages
#
sudo apt-get install -y --no-install-recommends \
		  software-properties-common \
		  apt-transport-https \
		  ca-certificates \
		  gnupg
		  	  
sudo wget -qO - https://apt.kitware.com/keys/kitware-archive-latest.asc | sudo apt-key add - && \
    sudo apt-add-repository 'deb https://apt.kitware.com/ubuntu/ bionic main' && \
    sudo apt-get update && \
    sudo apt-get install -y --no-install-recommends --only-upgrade \
    cmake
    
cmake --version

# 
# download/build ROS from source
#
sudo mkdir -p /opt/ros
sudo chown $USER:$USER /opt/ros

mkdir -p ${ROS_ROOT}/src && \
    cd ${ROS_ROOT} && \
    
    # https://answers.ros.org/question/325245/minimal-ros2-installation/?answer=325249#post-id-325249
    rosinstall_generator --deps --rosdistro ${ROS_DISTRO} ${ROS_PKG} \
		launch_xml \
		launch_yaml \
		launch_testing \
		launch_testing_ament_cmake \
		demo_nodes_cpp \
		demo_nodes_py \
		example_interfaces \
		camera_calibration_parsers \
		camera_info_manager \
		cv_bridge \
		v4l2_camera \
		vision_opencv \
		vision_msgs \
		image_geometry \
		image_pipeline \
		image_transport \
		compressed_image_transport \
		compressed_depth_image_transport \
		> ros2.${ROS_DISTRO}.${ROS_PKG}.rosinstall && \
    cat ros2.${ROS_DISTRO}.${ROS_PKG}.rosinstall && \
    vcs import src < ros2.${ROS_DISTRO}.${ROS_PKG}.rosinstall && \

    # install dependencies using rosdep
    cd ${ROS_ROOT} && \
    rosdep init && \
    rosdep update && \
    rosdep install -y \
        --ignore-src \
        --from-paths src \
	  --rosdistro ${ROS_DISTRO} \
	  --skip-keys "libopencv-dev libopencv-contrib-dev libopencv-imgproc-dev python-opencv python3-opencv" && \
    # build it!
    colcon build \
        --merge-install \
        --cmake-args -DCMAKE_BUILD_TYPE=Release
    
