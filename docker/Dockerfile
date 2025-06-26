FROM osrf/ros:humble-desktop-jammy

ARG BRANCH_NAME=main
ENV STATE_SHARING_PORT=16800
ENV STATE_SHARING_INTERFACE=enp71s0
RUN /bin/bash -c "echo source /opt/ros/humble/setup.bash" >> ~/.bashrc
COPY dds_topics.yaml /tmp/dds_topics.yaml
COPY requirements.txt /tmp/requirements.txt
RUN curl -sSL https://raw.githubusercontent.com/eProsima/vulcanexus/main/vulcanexus.key -o /usr/share/keyrings/vulcanexus-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/vulcanexus-archive-keyring.gpg] http://repo.vulcanexus.org/debian/ jammy main" | tee -a /etc/apt/sources.list.d/vulcanexus.list > /dev/null

RUN apt-get update && apt-get -y upgrade && \
    apt-get -y install \
    vim \
    nano \
    tmux \
    tmuxinator \
    iputils-ping \
    openssh-client \
    wget

RUN apt-get -y install \
         vulcanexus-humble-desktop \
;

RUN curl -sS https://bootstrap.pypa.io/get-pip.py | python3
RUN pip3 install -r /tmp/requirements.txt && rm /tmp/requirements.txt
RUN mkdir -p -m 0700 ~/.ssh && ssh-keyscan bitbucket.org >> ~/.ssh/known_hosts
RUN --mount=type=ssh git clone git@bitbucket.org:autonomouscv/gnc_px4_autopilot.git /setup_repos/PX4-Autopilot
RUN --mount=type=ssh  cd /setup_repos/PX4-Autopilot && git checkout -f ${BRANCH_NAME} && git submodule update --init --recursive
RUN sed -i 's/matplotlib>=3\.0\.\*/matplotlib/g' /setup_repos/PX4-Autopilot/Tools/setup/requirements.txt
RUN /bin/bash -c /setup_repos/PX4-Autopilot/Tools/setup/ubuntu.sh
RUN cp -f /tmp/dds_topics.yaml /setup_repos/PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.yaml
RUN --mount=type=ssh  /bin/bash -c "cd /setup_repos/PX4-Autopilot && make px4_sitl"
RUN rm -rf /setup_repos/gz_models/

RUN mkdir /setup_repos/uros_ws

RUN --mount=type=ssh /bin/bash -c "source /opt/ros/humble/setup.bash; \
    cd /setup_repos/uros_ws; \
    git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup; \
    rosdep update && rosdep install --from-paths src --ignore-src -y; \
    git clone git@bitbucket.org:autonomouscv/gnc_px4_msgs.git src/px4_msgs; cd src/px4_msgs; \
    git checkout -f ${BRANCH_NAME}; cd ../..; \
    colcon build; \
    source install/local_setup.bash; \
    ros2 run micro_ros_setup create_agent_ws.sh; \
    ros2 run micro_ros_setup build_agent.sh; \
    source install/local_setup.sh"

RUN /bin/bash -c 'echo mavlink start  \
-u \$STATE_SHARING_PORT \
-r 400000 \
-m state_sharing \
-o \$STATE_SHARING_PORT \
-p \
-n \$STATE_SHARING_INTERFACE' \
>> /setup_repos/PX4-Autopilot/build/px4_sitl_default/etc/init.d-posix/px4-rc.mavlink

RUN /bin/bash -c 'echo state_sharing start'  \
>> /setup_repos/PX4-Autopilot/build/px4_sitl_default/etc/init.d-posix/px4-rc.mavlink
 
RUN sudo apt install ros-humble-ros-gzharmonic -y

RUN /bin/bash -c "echo source /setup_repos/uros_ws/install/local_setup.sh" >> ~/.bashrc
RUN /bin/bash -c "git config --global --add safe.directory '*'"
COPY tmux.conf /root/.tmux.conf