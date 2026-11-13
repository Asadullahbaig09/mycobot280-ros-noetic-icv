# Use an NVIDIA base image with CUDA and OpenGL support for your RTX 4060
FROM nvidia/opengl:1.1-glvnd-runtime-ubuntu20.04

# Set up ROS Noetic Sources
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    lsb-release \
    locales \
    gnupg \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*

RUN echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list
RUN mkdir -p /usr/share/keyrings
# Download the key, process it with gpg, and place it in the keyrings directory
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | gpg --dearmor > /usr/share/keyrings/ros-archive-keyring.gpg

# Install ROS Noetic Desktop Full
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop-full \
    python3-rosdep python3-pip python3-catkin-tools \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# rosdep must be initialised once, otherwise `rosdep install` below fails
RUN rosdep init && rosdep update

# Install myCobot Python API
RUN pip3 install pymycobot

# Set up Catkin workspace
WORKDIR /home/mycobot_ws
RUN mkdir -p src

# Copy project files into the container (.git etc. excluded via .dockerignore)
COPY . /home/mycobot_ws/src/mycobot_ros

# Build the Workspace
RUN bash -c "source /opt/ros/noetic/setup.bash && \
    cd /home/mycobot_ws && \
    apt-get update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/* && \
    catkin_make"

# ROS env for both non-interactive (docker-compose `command`) and interactive shells.
# /etc/bash.bashrc alone is skipped by non-interactive bash, hence BASH_ENV.
RUN printf 'source /opt/ros/noetic/setup.bash\nsource /home/mycobot_ws/devel/setup.bash\n' > /etc/ros_env.sh \
    && echo "source /etc/ros_env.sh" >> /etc/bash.bashrc
ENV BASH_ENV=/etc/ros_env.sh

# Fix the QT bug for Rviz/Gazebo display
ENV QT_X11_NO_MITSHM=1

# docker-compose passes the launch command as a single string, so run it via bash -c
ENTRYPOINT ["/bin/bash", "-c"]
CMD ["bash"]
