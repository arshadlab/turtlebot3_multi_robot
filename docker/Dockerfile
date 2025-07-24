# Base image with ROS 2 Jazzy Desktop
FROM osrf/ros:jazzy-desktop-full

# Environment setup
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy

# Install basic build dependencies and tools
RUN apt-get update && apt-get install -y \
    git \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    curl \
    ros-jazzy-rmw-implementation \
    ros-jazzy-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# Create your CycloneDDS config file
RUN echo '<CycloneDDS>\n\
  <Discovery>\n\
    <ParticipantIndex>auto</ParticipantIndex>\n\
    <MaxAutoParticipantIndex>100</MaxAutoParticipantIndex>\n\
  </Discovery>\n\
</CycloneDDS>' > /etc/cyclonedds.xml

# Set CYCLONEDDS_URI
ENV CYCLONEDDS_URI=file:///etc/cyclonedds.xml

# Initialize rosdep
RUN rosdep update

# Create non-root accessible workspace
WORKDIR /opt/ros2_ws
RUN mkdir -p src && chmod -R a+rwx /opt/ros2_ws

# Clone your repository into src
RUN git clone https://github.com/arshadlab/tb3_multi_robot.git src/tb3_multi_robot -b jazzy

# Install dependencies using rosdep
RUN rosdep install --from-paths src --ignore-src -r -y

# Source ROS environment before build
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build --symlink-install"

# Make sure workspace is readable/executable by all users
RUN chmod -R a+rX /opt/ros2_ws

# Default shell setup when container starts
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN echo "source /opt/ros2_ws/install/setup.bash" >> ~/.bashrc

# Copy entrypoint script and make it executable
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Use it as entrypoint
ENTRYPOINT ["/entrypoint.sh"]

# Default workdir inside container
WORKDIR /opt/ros2_ws

