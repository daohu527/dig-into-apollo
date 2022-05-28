# 如何编译

> 高行微言，所以修身。


<a name="build" />

## 编译
编译脚本在"apollo.sh"中实现，通过shell脚本设置一些参数和环境变量，最后通过bazel编译。下面我们分析下apollo.sh的具体实现。


"apollo.sh"中实现了一些函数，我们先介绍下build函数
#### build
```
function build() {
  if [ "${USE_GPU}" = "1" ] ; then
    echo -e "${YELLOW}Running build under GPU mode. GPU is required to run the build.${NO_COLOR}"
  else
    echo -e "${YELLOW}Running build under CPU mode. No GPU is required to run the build.${NO_COLOR}"
  fi
  info "Start building, please wait ..."

  generate_build_targets
  info "Building on $MACHINE_ARCH..."

  MACHINE_ARCH=$(uname -m)
  JOB_ARG="--jobs=$(nproc) --ram_utilization_factor 80"
  if [ "$MACHINE_ARCH" == 'aarch64' ]; then
    JOB_ARG="--jobs=3"
  fi
  info "Building with $JOB_ARG for $MACHINE_ARCH"

  bazel build $JOB_ARG $DEFINES -c $@ $BUILD_TARGETS
  if [ ${PIPESTATUS[0]} -ne 0 ]; then
    fail 'Build failed!'
  fi

  # Build python proto
  build_py_proto

  # Clear KV DB and update commit_id after compiling.
  if [ "$BUILD_FILTER" == 'cyber' ] || [ "$BUILD_FILTER" == 'drivers' ]; then
    info "Skipping revision recording"
  else
    bazel build $JOB_ARG $DEFINES -c $@ $BUILD_TARGETS
    if [ ${PIPESTATUS[0]} -ne 0 ]; then
      fail 'Build failed!'
    fi
    rm -fr data/kv_db*
    REVISION=$(get_revision)
    ./bazel-bin/modules/common/kv_db/kv_db_tool --op=put \
        --key="apollo:data:commit_id" --value="$REVISION"
  fi

  if [ -d /apollo-simulator ] && [ -e /apollo-simulator/build.sh ]; then
    cd /apollo-simulator && bash build.sh build
    if [ $? -ne 0 ]; then
      fail 'Build failed!'
    fi
  fi
  if [ $? -eq 0 ]; then
    success 'Build passed!'
  else
    fail 'Build failed'
  fi
}
```

<a name="question" />

## 常见问题
1. 如果机器内存小于8G，会出现编译错误的情况，单独编译又没有问题，问题的原因是内存缓冲不足，导致报错。调大内存后解决，错误信息如下。
```
gcc: internal compiler error: Killed
```

2. 因为一些文件需要下载之后才能编译，如果启动离线模式，或者想下载文件之后再进行编译，可以参考。
[bazel](https://docs.bazel.build/versions/0.18.1/external.html)