
#Install ROS2 Foxy
sudo apt-get -y install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
sudo apt update
sudo apt-get -y install bzip2 ca-certificates ccache cmake \
		cppcheck dirmngr doxygen file g++ gcc gdb git \
		gnupg gnupg2 gosu lcov libfreetype6-dev libgtest-dev \
		libpng-dev libssl-dev lsb-release make ninja-build \
		openjdk-8-jdk openjdk-8-jre openssh-client \
		pkg-config python3-dev python3-pip \
		rsync shellcheck tzdata unzip valgrind \
		wget xsltproc zip ant binutils bc dirmngr \
		gazebo11 gstreamer1.0-plugins-bad gstreamer1.0-plugins-base \
		gstreamer1.0-plugins-good gstreamer1.0-plugins-ugly \
		libeigen3-dev libgazebo11-dev libgstreamer-plugins-base1.0-dev \
		libimage-exiftool-perl libopencv-dev \
		libxml2-utils mesa-utils protobuf-compiler \
		x-window-system geographiclib-tools libeigen3-dev \
		libgeographic-dev libopencv-dev libyaml-cpp-dev \
		python3-colcon-common-extensions xvfb \
		autoconf automake bison build-essential \
		bzip2 file flex genromfs gperf libncurses-dev \
		libtool pkg-config uncrustify vim-common python3-rosdep \
		ros-foxy-ros-base ros-foxy-desktop ros-foxy-cv-bridge \
		ros-foxy-image-tools ros-foxy-image-transport ros-foxy-image-transport-plugins \
		ros-foxy-image-pipeline ros-foxy-camera-calibration-parsers \
		ros-foxy-camera-info-manager ros-foxy-launch-testing-ament-cmake \
		ros-foxy-cv-bridge ros-foxy-vision-opencv \
		ros-foxy-gazebo-dev ros-foxy-gazebo-msgs ros-foxy-gazebo-msgs-dbgsym \
		ros-foxy-gazebo-plugins ros-foxy-gazebo-plugins-dbgsym ros-foxy-gazebo-ros \
		ros-foxy-gazebo-ros2-control ros-foxy-gazebo-ros2-control-dbgsym \
		ros-foxy-gazebo-ros2-control-demos ros-foxy-gazebo-ros2-control-demos-dbgsym \
		ros-foxy-gazebo-ros-dbgsym ros-foxy-gazebo-ros-pkgs ros-foxy-vision-msgs \
		ros-foxy-vision-msgs-dbgsym xterm python3-pysolar

sudo geographiclib-get-geoids egm96-5
sudo apt-get -y autoremove

source /opt/ros/foxy/setup.bash

# Install Python 3 pip build dependencies first.
sudo apt apt install python3-argcomplete python3-empy python3-jinja2 python3-cerberus python3-coverage python3-matplotlib python3-numpy python3-packaging python3-pkgconfig python3-opencv python3-colcon-common-extensions python3-wheel python3-setuptools python3-request python3-serial python3-six python3-toml python3-psutil

python3 -m pip install --upgrade pip wheel setuptools

# Python 3 dependencies installed by pip
python3 -m pip install nunavut pyros-genmsg pyulog pyyaml

echo 'source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash' >> ~/.bashrc

# manual ccache setup
sudo ln -s /usr/bin/ccache /usr/lib/ccache/cc \
	&& ln -s /usr/bin/ccache /usr/lib/ccache/c++

# astyle v3.1
wget -q https://downloads.sourceforge.net/project/astyle/astyle/astyle%203.1/astyle_3.1_linux.tar.gz -O /tmp/astyle.tar.gz \
	&& cd /tmp && tar zxf astyle.tar.gz && cd astyle/src \
	&& sudo make -f ../build/gcc/Makefile -j$(nproc) \
	&& sudo cp bin/astyle /usr/local/bin

# Gradle (Required to build Fast-RTPS-Gen)
wget -q "https://services.gradle.org/distributions/gradle-6.3-rc-4-bin.zip" -O /tmp/gradle-6.3-rc-4-bin.zip
sudo mkdir -p /opt/gradle
cd /tmp
sudo unzip -d /opt/gradle gradle-6.3-rc-4-bin.zip

export PATH="/opt/gradle/gradle-6.3-rc-4/bin:$PATH"

# Intall foonathan_memory from source as it is required to Fast-RTPS >= 1.9
git clone https://github.com/eProsima/foonathan_memory_vendor.git /tmp/foonathan_memory
cd /tmp/foonathan_memory
mkdir build && cd build
cmake ..
sudo cmake --build . --target install -- -j $(nproc)
