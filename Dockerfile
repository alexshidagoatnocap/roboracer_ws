FROM ros:foxy

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y \
    git \
    vim \
    nano \
    tmux \
    ros-foxy-ackermann-msgs \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /roboracer_ws
CMD [ "/bin/bash", "-c", "source /opt/ros/foxy/setup.bash && exec bash" ]