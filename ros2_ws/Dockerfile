# robotic_hand_ws/Dockerfile (ROS 2 Humble 테스트 베드) 기반 + 3조 실기 드라이버 의존성
FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-serial \
    python3-pytest \
    git \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir smbus2 numpy pyyaml

WORKDIR /root/ros2_ws
COPY src ./src
RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /root/ros2_ws/install/setup.bash" >> /root/.bashrc

CMD ["bash"]
