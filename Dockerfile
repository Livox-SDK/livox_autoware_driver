FROM autoware/autoware:latest-melodic

ENV USERNAME autoware

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Build Livox_SDK
RUN bash -c 'mkdir -p /home/$USERNAME/Livox_SDk; \
    cd /home/$USERNAME/Livox_SDk; \
    git clone https://github.com/Livox-SDK/Livox-SDK.git; \
    cd Livox-SDK/build; \
    cmake ..; \
    make; \
    sudo make install; \
    rm -rf /home/$USERNAME/Livox_SDk'

COPY ./scripts/ /home/$USERNAME/Autoware/src/autoware/utilities/runtime_manager/scripts/

# Build Livox_ros_driver
COPY . /home/$USERNAME/Autoware/src/drivers/awf_drivers/livox_ros_driver/

RUN bash -c 'cd /home/$USERNAME/Autoware; \
    source /opt/ros/$ROS_DISTRO/setup.bash; \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release'

