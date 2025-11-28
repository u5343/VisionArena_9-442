FROM vision-vrena-2025:v0.1.2


COPY ./install /home/va/Vision-Vrena-2025/install
COPY ./src/player_pkg /home/va/Vision-Vrena-2025/src/player_pkg
COPY ./results /home/va/Vision-Vrena-2025/results


# 将选手所需程序移入镜像
WORKDIR /home/va/Vision-Vrena-2025
# 设置工作空间
RUN apt-get update && apt-get upgrade -y && mkdir -p src/referee_pkg/results && apt-get install -y ros-humble-xacro ros-humble-gazebo-ros-pkgs
# 下载一些必要的包