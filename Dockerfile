FROM ubuntu:22.04 as localsetup

RUN apt update && apt install locales
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

FROM localsetup as dependenciesros

RUN --mount=type=cache,target=/root/.cache apt update && apt install -y curl gnupg2 lsb-release
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2-latest.list'

FROM dependenciesros as dependenciesfastdds

ENV DEBIAN_FRONTEND="noninteractive"

RUN --mount=type=cache,target=/root/.cache apt update && apt install -y \
  build-essential \
  git \
  python3-pip \
  python3-rosdep \
  python3-vcstool \
  python3-numpy \
  python3-empy \
  libxml2-utils \
  python3-pytest \ 
  libcurl3-nss \
  python3-pytest-cov \
  wget \
  libasio-dev \
  libtinyxml2-dev \
  libcunit1-dev \
  libssl-dev \
  libp11-dev \
  libengine-pkcs11-openssl \
  softhsm2 \
  libengine-pkcs11-openssl \
  swig \
  libpython3-dev \
  libxaw7-dev \
  libeigen3-dev \
  acl-dev \
  python3-sip-dev \
  libopencv-dev \
  libbullet-dev \
  graphviz \
  clang-format \
  python3-nose \
  libconsole-bridge-dev \
  cmake \
  libignition-cmake2-dev \
  doxygen \
  libgtest-dev \
  python3-flake8 \
  libbenchmark-dev \
  libtinyxml-dev \
  libyaml-cpp-dev \
  python3-argcomplete \
  python3-packaging \
  python3-pytest-timeout \
  python3-psutil \
  liborocos-kdl1.5 \
  libzstd-dev \
  libignition-math6-dev \
  libpyside2-dev \
  libshiboken2-dev \
  shiboken2 \
  python3-lxml \
  python3-cryptography \
  pydocstyle \
  python3-pycodestyle \
  python3-matplotlib \
  python3-mypy \
  python3-pytest-mock \
  python3-pydot \
  python3-pygraphviz \
  libyaml-dev \
  clang-tidy \
  libgl1-mesa-dev \
  python3-lark \
  libbabeltrace1 \
  python3-lttng \
  python3-netifaces \
  libcurl4-openssl-dev \
  qtbase5-dev \
  libqt5charts5-dev

RUN --mount=type=cache,target=/root/.cache/pip python3 -m pip install -U \
  argcomplete \
  flake8 \
  flake8-blind-except \
  flake8-builtins \
  flake8-class-newline \
  flake8-comprehensions \
  flake8-deprecated \
  flake8-docstrings \
  flake8-import-order \
  flake8-quotes \
  pytest-repeat \
  pytest-rerunfailures \
  pytest \
  pytest-cov \
  pytest-runner \
  setuptools \
  colcon-common-extensions

FROM dependenciesfastdds as qtdependencies

RUN --mount=type=cache,target=/root/.cache apt update && apt install -y \
  libx11-dev \
  libxrandr-dev \
  qtdeclarative5-dev \
  qtquickcontrols2-5-dev \
  libfreetype6-dev
  #libqt5qml5
  #pyqt5-dev
  #python3-pyqt5
  #libqt5qml5 \

RUN --mount=type=cache,target=/root/.cache apt-get update && apt-get install -y \
  qml-module-qtquick-dialogs \
  qml-module-qtquick-controls \
  qml-module-qtquick-controls2 \
  qml-module-qtgraphicaleffects \
  qml-module-qtcharts \
  qml-module-qtquick-layouts \
  qml-module-qt-labs-calendar

RUN --mount=type=cache,target=/root/.cache/pip python3 -m pip install -U \
  PyQt5

ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV FASTDDS_STATISTICS="HISTORY_LATENCY_TOPIC;NETWORK_LATENCY_TOPIC;PUBLICATION_THROUGHPUT_TOPIC;\
  SUBSCRIPTION_THROUGHPUT_TOPIC;RTPS_SENT_TOPIC;RTPS_LOST_TOPIC;\
  HEARTBEAT_COUNT_TOPIC;ACKNACK_COUNT_TOPIC;NACKFRAG_COUNT_TOPIC;\
  GAP_COUNT_TOPIC;DATA_COUNT_TOPIC;RESENT_DATAS_TOPIC;SAMPLE_DATAS_TOPIC;\
  PDP_PACKETS_TOPIC;EDP_PACKETS_TOPIC;DISCOVERY_TOPIC;PHYSICAL_DATA_TOPIC"

#--cmake-args -DFASTDDS_STATISTICS=ON