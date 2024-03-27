FROM osrf/ros:iron-desktop
ARG USER_ID=1000
ARG GROUP_ID=1000

#  install dependencies
RUN apt-get update && apt-get install --no-install-recommends -y \
  wget \
  unzip \
  openjdk-8-jdk \
  vim \
  python3-pip \
  terminator \
  bash-completion \
  curl \
  libcanberra-gtk3-module \
  && apt-get -y autoremove \
  && apt-get clean


ARG USER=docker
# set user as default so install scripts look the same for local host install
RUN addgroup --gid $GROUP_ID ${USER} && \
  adduser --disabled-password --gecos '' --uid $USER_ID --gid $GROUP_ID ${USER} && \
  adduser ${USER} sudo && \
  echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
USER ${USER}

RUN sudo apt-get update && \
    curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
ENV PATH=$HOME/.cargo/env:$PATH

RUN mkdir ~/ws_rmw_zenoh/src -p && cd ~/ws_rmw_zenoh/src && \
    git clone https://github.com/ros2/rmw_zenoh.git && \
    cd ~/ws_rmw_zenoh && \
    . /opt/ros/$ROS_DISTRO/setup.sh && \
    sudo rm -rf /etc/ros/rosdep/sources.list.d/20-default.list && \
    sudo rosdep init && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    . /opt/ros/$ROS_DISTRO/setup.sh && \
    . $HOME/.cargo/env && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# extra packages
RUN sudo apt-get update && sudo apt-get install -y \
    tmux \
    tmuxinator

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

# vim: set et fenc=utf-8 ff=unix ft=dockerfile sts=0 sw=2 ts=2 :
