FROM osrf/ros:melodic-desktop-full-bionic

# Set default shell
SHELL ["/bin/bash", "-c"]

ARG USER=user
ARG UID=1000
ARG GID=1000
ARG PW=user@123

RUN groupadd -g ${GID} -o ${USER}
RUN useradd --system --create-home --home-dir /home/${USER} --shell /bin/bash --uid ${UID} -g ${GID} --groups sudo,video ${USER} && \ 
    echo "${USER}:${PW}" | chpasswd && \
    echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

ENV QT_X11_NO_MITSHM=1 \
    USER=${USER} \
    LANG=en_US.UTF-8 \
    HOME=/home/${USER} \
    XDG_RUNTIME_DIR=/run/user/${UID} \
    TZ=America/New_York 


USER ${USER}
WORKDIR ${HOME}
# custom Bash prompt
RUN { echo && echo "PS1='\[\e]0;\u \w\a\]\[\033[01;32m\]\u\[\033[00m\] \[\033[01;34m\]\w\[\033[00m\] \\\$ '" ; } >> .bashrc

RUN sudo mkdir -p -m 0700 /run/user/${UID} && \
    sudo chown ${USER}:${USER} /run/user/${UID}

# Basic setup
RUN sudo apt-get update && sudo apt-get install -y --no-install-recommends --allow-unauthenticated \
    software-properties-common \
    build-essential \
    g++ \
    git \
    ca-certificates \
    make \
    automake \
    autoconf \
    libtool \
    pkg-config \
    python \
    libxext-dev \
    libx11-dev \
    doxygen \
    tmux \
    tzdata \
    xclip \
    mc \
    curl \
    iproute2 \
    iputils-ping \   
    x11proto-gl-dev && \
    sudo rm -rf /var/lib/apt/lists/* 

# Setup tmux config
ADD --chown=${USER}:${USER} https://raw.githubusercontent.com/kanishkaganguly/dotfiles/master/tmux/.tmux.bash.conf $HOME/.tmux.conf

# Set datetime and timezone correctly
# Remove duplicate sources
RUN sudo ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo '$TZ' | sudo tee -a /etc/timezone

RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -


# Install ROS packages
RUN sudo apt-get update && sudo apt-get install -y \
    python-catkin-tools \
    ros-melodic-moveit-visual-tools \
    ros-melodic-moveit-* \
    ros-melodic-usb-cam \
    ros-melodic-fiducial-msgs \
    ros-melodic-aruco-detect \
    # ros-melodic-gripper-action-controller \
    libeigen3-dev \
    python-pymodbus \
    python-serial \
    ros-melodic-soem

# Setup ROS workspace directory
RUN mkdir -p $HOME/catkin_ws/src && \
    catkin init --workspace $HOME/catkin_ws/ && \
    cd $HOME/catkin_ws/src

# cmake 3.16
ADD https://cmake.org/files/v3.16/cmake-3.16.9-Linux-x86_64.sh /opt/cmake-3.16.9-Linux-x86_64.sh
WORKDIR /opt/
RUN sudo chmod +x /opt/cmake-3.16.9-Linux-x86_64.sh && \
    bash -c "yes Y | sudo /opt/cmake-3.16.9-Linux-x86_64.sh" && \
    bash -c "sudo ln -s /opt/cmake-3.16.9-Linux-x86_64/bin/* /usr/local/bin"

# Set up ROS
RUN source /opt/ros/melodic/setup.bash && \
    cd ${HOME}/catkin_ws && \
    catkin build && \
    source $HOME/catkin_ws/devel/setup.bash

# Setup UR Drivers
RUN source /opt/ros/melodic/setup.bash && \
    cd ~/catkin_ws && \
    sudo apt update -qq && \
    rosdep update --include-eol-distros && \
    rosdep install --from-paths src --ignore-src -y

# Set up working directory and bashrc
WORKDIR ${HOME}/catkin_ws/
RUN echo 'source /opt/ros/melodic/setup.bash' >> $HOME/.bashrc && \
    echo 'source $HOME/catkin_ws/devel/setup.bash' >> $HOME/.bashrc
CMD /bin/bash
