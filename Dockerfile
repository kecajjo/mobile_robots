FROM osrf/ros:noetic-desktop-full

USER root
    RUN useradd -m developer && \
        echo "developer:123" | chpasswd && \
        adduser developer sudo
    RUN apt-get update && apt-get install -y \
        python3-tk && \
        rm -rf /var/lib/apt/lists/*

USER developer
    RUN mkdir -p /home/developer/ws/src
    RUN echo "source /opt/orocos/melodic/setup.bash" >> /home/developer/.bashrc
    WORKDIR /home/developer/ws

ENTRYPOINT ["/bin/bash", "-c"]