FROM ros:humble

RUN apt update && apt install -y --no-install-recommends gnupg

RUN apt update && apt -y upgrade

RUN apt update && apt install -y --no-install-recommends \
        meson \
	ninja-build \
	pkg-config \
	libyaml-dev \
	python3-yaml \
	python3-ply \
	python3-jinja2 \
	libevent-dev \
	libdrm-dev \
	libcap-dev \
	python3-pip \
	python3-opencv \
     && apt-get clean \
     && apt-get autoremove \
     && rm -rf /var/cache/apt/archives/* \
     && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# Install libcamera from source
RUN git clone https://github.com/raspberrypi/libcamera.git && cd libcamera && git checkout 6ddd79b && cd ..
RUN meson setup libcamera/build libcamera/
RUN ninja -C libcamera/build/ install


# Install kmsxx from source
RUN git clone https://github.com/tomba/kmsxx.git
RUN meson setup kmsxx/build kmsxx/
RUN ninja -C kmsxx/build/ install 

# Add the new installations to the python path so that picamera2 can find them
ENV PYTHONPATH=$PYTHONPATH:/usr/local/lib/aarch64-linux-gnu/python3.10/site-packages:/app/kmsxx/build/py

# Finally install picamera2 using pip
RUN pip3 install picamera2

# Set ROS_DOMAIN_ID environment variable
ENV ROS_DOMAIN_ID=50

# Install additional ROS2 dependencies and TurtleBot3 dependencies
RUN apt update && apt install -y --no-install-recommends \
    python3-cv-bridge \
    python3-argcomplete \
    python3-colcon-common-extensions \
    libboost-system-dev \
    build-essential \
    libudev-dev \
    && apt-get clean \
    && apt-get autoremove \
    && rm -rf /var/cache/apt/archives/* \
    && rm -rf /var/lib/apt/lists/*

# Install ROS2 development tools (ament-cmake is already included in ros:humble)
RUN apt update && apt install -y --no-install-recommends \
    build-essential \
    cmake \
    python3-pip \
    python3-setuptools \
    && apt-get clean \
    && apt-get autoremove \
    && rm -rf /var/cache/apt/archives/* \
    && rm -rf /var/lib/apt/lists/*

# Install ROS2 packages
RUN apt update && apt install -y --no-install-recommends \
    ros-humble-rclcpp \
    ros-humble-rclpy \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    ros-humble-tf2 \
    ros-humble-tf2-ros \
    ros-humble-nav-msgs \
    ros-humble-std-msgs \
    && apt-get clean \
    && apt-get autoremove \
    && rm -rf /var/cache/apt/archives/* \
    && rm -rf /var/lib/apt/lists/*

# Install TurtleBot3 specific packages
RUN apt update && apt install -y --no-install-recommends \
    ros-humble-hls-lfcd-lds-driver \
    ros-humble-turtlebot3-msgs \
    ros-humble-dynamixel-sdk \
    ros-humble-xacro \
    && apt-get clean \
    && apt-get autoremove \
    && rm -rf /var/cache/apt/archives/* \
    && rm -rf /var/lib/apt/lists/*

# Copy the test script to the container
COPY camera_test /app/camera_test

# Setup TurtleBot3 workspace (build later in container)
RUN mkdir -p /app/turtlebot3_ws/src && cd /app/turtlebot3_ws/src
RUN git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3.git
RUN git clone -b humble https://github.com/ROBOTIS-GIT/ld08_driver.git
RUN git clone -b humble https://github.com/ROBOTIS-GIT/coin_d4_driver
# Add RPLIDAR A1M8 driver
RUN git clone https://github.com/Slamtec/sllidar_ros2.git
# Add Dynamixel SDK from source
RUN git clone -b humble https://github.com/ROBOTIS-GIT/DynamixelSDK.git
RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc

# Set default command to keep container running
CMD ["bash"]
