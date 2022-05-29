# 启动容器

> 橘生淮南则为橘，生于淮北则为枳。


docker的主要的好处是开箱即用，在编译docker的时候安装好需要的环境，在使用的时候就无需担心环境问题带来的影响了。下面我们主要分析下docker文件夹中的脚本，主要涉及docker的编译、启动、以及host相关的内容。


## docker编译
编译docker在目录中`apollo\docker\build`中，执行的命令为
```sh
./build_dev.sh ./dev.x86_64.dockerfile
```
在"build_dev.sh"脚本中会执行编译docker的工作，下面我们分析下docker的编译过程。

#### build_dev.sh
```sh
# Usage:
#   ./build_dev.sh ./dev.x86_64.dockerfile
# 1. 获取dockerfile名称，这里为第一个参数
DOCKERFILE=$1

# 2. 获取build_dev.sh的目录路径
CONTEXT="$(dirname "${BASH_SOURCE[0]}")"

REPO=apolloauto/apollo
ARCH=$(uname -m)
TIME=$(date +%Y%m%d_%H%M)

TAG="${REPO}:dev-18.04-${ARCH}-${TIME}"

# Fail on first error.
set -e
# 3. 编译docker
docker build -t ${TAG} -f ${DOCKERFILE} ${CONTEXT}
echo "Built new image ${TAG}"
```
其中解释下几个参数：
-t : 设置docker的tag名称
-f : 默认不需要设置这个参数，docker会从当前目录中找dockerfile编译，当有多个dockerfile的时候就需要通过"-f"来指定编译的dockerfile
CONTEXT ： docker编译的资源目录

[参考](https://docs.docker.com/engine/reference/commandline/build/)

**疑问**
1. ARM架构和X86_64架构的docker编译有什么区别？


接着我们来看dockerfile
#### dev.x86_64.dockerfile
```sh
# 1. 以nvidia/cuda:10.0做为基础镜像
FROM nvidia/cuda:10.0-cudnn7-devel-ubuntu18.04

ENV DEBIAN_FRONTEND=noninteractive

# 2. 安装软件
RUN apt-get update -y && \
    apt-get install -y \
    apt-transport-https \
    autotools-dev \
    automake \
    bc \
    build-essential \
    cmake \
    cppcheck \
    curl \
    curlftpfs \
    debconf-utils \
    doxygen \
    gdb \
    git \
    google-perftools \
    graphviz \
    iproute2 \
    iputils-ping \
    lcov \
    libblas-dev \
    libssl-dev \
    libboost-all-dev \
    libcurl4-openssl-dev \
    libfreetype6-dev \
    liblapack-dev \
    libpcap-dev \
    libsqlite3-dev \
    libgtest-dev \
    locate \
    lsof \
    nfs-common \
    python-autopep8 \
    shellcheck \
    software-properties-common \
    sshfs \
    subversion \
    unzip \
    uuid-dev \
    v4l-utils \
    vim \
    wget \
    libasound2-dev \
    zip && \
    apt-get clean && rm -rf /var/lib/apt/lists/* && \
    echo '\n\n\n' | ssh-keygen -t rsa

# Run installers.
# 3. 拷贝installers中的脚本，并且执行
COPY installers /tmp/installers
# 4. 安装adv，作用？？？
RUN bash /tmp/installers/install_adv_plat.sh
# 5. 安装bazel
RUN bash /tmp/installers/install_bazel.sh
# 6. 安装bazel依赖的包
RUN bash /tmp/installers/install_bazel_packages.sh
# 7. 网络文件系统，百度bosfs基于libfuse
RUN bash /tmp/installers/install_bosfs.sh
# 8. 安装conda，包管理软件
RUN bash /tmp/installers/install_conda.sh
# 9. 安装ffmpeg，用于视频处理
RUN bash /tmp/installers/install_ffmpeg.sh
# 10. 安装gflag
RUN bash /tmp/installers/install_gflags_glog.sh
# 11. openGL扩展
RUN bash /tmp/installers/install_glew.sh
# 12. google代码规范
RUN bash /tmp/installers/install_google_styleguide.sh
# 13. 安装caffe
RUN bash /tmp/installers/install_gpu_caffe.sh
# 14. 连续系统的大规模非线性优化的软件库
RUN bash /tmp/installers/install_ipopt.sh
# 15. osqp求解器
RUN bash /tmp/installers/install_osqp.sh
# 16. json rpc调用，是哪个开源库没有备注
RUN bash /tmp/installers/install_libjsonrpc-cpp.sh
# 17. nonlinear optimization非线性优化
RUN bash /tmp/installers/install_nlopt.sh
# 18. node.js版本管理
RUN bash /tmp/installers/install_node.sh
# 19. 视频编解码库
RUN bash /tmp/installers/install_openh264.sh
# 20. ota安全包，具体是？？？
RUN bash /tmp/installers/install_ota.sh
# 21. 安装pcl点云库
RUN bash /tmp/installers/install_pcl.sh
# 22. poco提供快速的可移植的网络开发
RUN bash /tmp/installers/install_poco.sh
# 23. 安装protobuf
RUN bash /tmp/installers/install_protobuf.sh
# 24. 安装python模块
RUN bash /tmp/installers/install_python_modules.sh
# 25. qp库
RUN bash /tmp/installers/install_qp_oases.sh
# 26. 安装QT
RUN bash /tmp/installers/install_qt.sh
# 27.
RUN bash /tmp/installers/install_supervisor.sh
# 28. 2D图像的变换
RUN bash /tmp/installers/install_undistort.sh
# 29. 增加用户，并且添加初始化脚本
RUN bash /tmp/installers/install_user.sh
# 30. 安装yarn，nodejs管理工具？？？
RUN bash /tmp/installers/install_yarn.sh
# 31. 性能调试库
RUN bash /tmp/installers/post_install.sh
# 32. 音频编解码
RUN bash /tmp/installers/install_opuslib.sh

# 33. 设置工作路径和用户为apollo
WORKDIR /apollo
USER apollo
```

还有一些没有用到的脚本
```sh
1.
install_adolc.sh

2. 安装fast-rtps，一个网络发现协议
install_fast-rtps.sh

3. 安装pytorch
install_libtorch.sh
```


## docker脚本
容器相关的脚本在`scripts`中，下面我逐步分析下这些脚本做了哪些工作。

#### dev_start.sh
启动容器的脚本

#### dev_into.sh
进入容器的脚本


## 设置主机
设置host，提供了一些在主机上使用的脚本

#### cleanup_resources.sh
清楚宿主机上的docker镜像，卷

#### install_docker.sh & install_nvidia_docker.sh
安装docker

#### setup_host.sh
```sh
APOLLO_ROOT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/../.." && pwd )"

# Setup core dump format.
if [ -e /proc/sys/kernel ]; then
  echo "${APOLLO_ROOT_DIR}/data/core/core_%e.%p" | \
      sudo tee /proc/sys/kernel/core_pattern
fi

# 设置每1分钟进行NTP时间同步
# Setup ntpdate to run once per minute. Log at /var/log/syslog.
grep -q ntpdate /etc/crontab
if [ $? -ne 0 ]; then
  echo "*/1 * * * * root ntpdate -v -u us.pool.ntp.org" | \
      sudo tee -a /etc/crontab
fi

# 使用udev管理设备文件
# Add udev rules.
sudo cp -r ${APOLLO_ROOT_DIR}/docker/setup_host/etc/* /etc/

#
# Add uvcvideo clock config.
grep -q uvcvideo /etc/modules
if [ $? -ne 0 ]; then
  echo "uvcvideo clock=realtime" | sudo tee -a /etc/modules
fi
```
