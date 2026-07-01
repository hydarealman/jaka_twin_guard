# JAKA Twin Guard — Dockerfile
# 共享于 方案A(WSL2原生Docker) 和 方案B(Docker Desktop)
#
# 构建:
#   docker build -t jaka-twin-guard:dev --target dev -f docker/Dockerfile .
#   docker build -t jaka-twin-guard:prod --target prod -f docker/Dockerfile .
#
# 基础镜像: ROS2 Humble + Gazebo 11 + RViz2 + 全部控制/感知依赖

FROM osrf/ros:humble-desktop-full AS deps
LABEL maintainer="hydarealman"
LABEL description="JAKA Twin Guard — dual-arm industrial manipulation (massage + carry + pick-place)"

# ============================================================
# 安装额外的 apt 依赖
# ============================================================
RUN apt-get update && apt-get install -y \
    # MoveIt2 全套
    ros-humble-moveit \
    ros-humble-moveit-msgs \
    ros-humble-moveit-configs-utils \
    ros-humble-moveit-planners-ompl \
    ros-humble-moveit-ros-control-interface \
    ros-humble-moveit-ros-move-group \
    ros-humble-moveit-ros-visualization \
    # ros2_control
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    # Gazebo ros2 集成
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    # Python 依赖
    python3-pip \
    python3-scipy \
    python3-numpy \
    # 软件渲染 (Mesa llvmpipe)
    mesa-utils \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    # 实用工具
    bash-completion \
    nano \
    && rm -rf /var/lib/apt/lists/*

# ============================================================
# 创建工作目录 + 非 root 用户 (可选, 方便权限管理)
# ============================================================
ARG USERNAME=rosuser
ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd --gid $USER_GID $USERNAME && \
    useradd --uid $USER_UID --gid $USER_GID -m $USERNAME && \
    echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

# 设置 rosuser 的 bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc && \
    echo "source /workspace/install/setup.bash 2>/dev/null" >> /home/$USERNAME/.bashrc

# ============================================================
# Stage: dev — 开发镜像 (源码通过 volume 挂载)
# ============================================================
FROM deps AS dev
WORKDIR /workspace

# entrypoint 和 launch 脚本在运行时挂载
# 源码在运行时通过 volume 挂载到 /workspace/src

USER $USERNAME

# ============================================================
# Stage: prod — 生产镜像 (源码 COPY 进镜像, 构建时编译)
# ============================================================
FROM deps AS prod
WORKDIR /workspace

# 复制源码
COPY src/ /workspace/src/
COPY docker/entrypoint.sh /entrypoint.sh
COPY docker/launch /launch

# 编译
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    cd /workspace && \
    colcon build --packages-select jaka_dual_arm dual_arm_jaka_c5_moveit_config jaka_c5_description"

# 清理 build 目录以减小镜像体积
RUN rm -rf /workspace/build

USER $USERNAME
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
