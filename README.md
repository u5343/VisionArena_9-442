# Vision_Arena_2025

视觉校内赛裁判系统，提供简单例程共参考

<font style="color:#DF2A3F;">**正式比赛运行时有所不同**</font>

# 一、启动方法
建议在非比赛阶段不使用docker进行工作
## 裁判系统
如有需要自行更改，下面仅作为示例
```
ros2 launch referee_pkg referee_pkg_launch.xml \
    TeamName:="TEAMENAME" \
    StageSelect:=0 \
    ModeSelect:=0
```
备赛期间选择起始阶段0，恒定模式0
## gazebo

运行摄像头仿真
```
ros2 launch camera_sim_pkg camera.launch.py
```
**比赛时不需要以下操作，仅运行摄像头仿真**
运行目标仿真
```
ros2 launch target_model_pkg target_action.launch.py
```

参数有：
model 模型文件（路径src/target_model_pkg/urdf/）

model_name 模型名字（不能生成名字一样的模型）<font style="color:#DF2A3F;">**识别要定义为[话题与服务消息说明](doc/Topic.md)下对应的模型名字**</font>(例如例程中使用的模型名字为sphere，这里的model_name也要写sphere)（**命名不同无法识别**）


x ：x位置

y ：y位置

z ：z位置

roll ：roll位置
例如(装甲板1)：
```
ros2 launch target_model_pkg target_action.launch.py model:=src/target_model_pkg/urdf/armor/armor_1.sdf model_name:=armor_1
```
默认是静止，如果运动的话
```
ros2 topic pub /type std_msgs/msg/Int32 "{data: 1}"
```
修改模型的位置
```
ros2 topic pub /pose geometry_msgs/msg/Pose "{position: {x: 1.0, y: 2.0, z: 0.5}}"
```
## 示例选手端测试程序
```
ros2 run player_pkg TestNode
```
## docker tar 文件使用
```
# 从tar文件中读取镜像 其名成为 vision-vrena-2025:v0.1.2
docker load -i Vision-Vrena-2025.tar

# 运行Dockerfile文件将选手的文件以及裁判系统文件移入以构造一个新的镜像其名称为vision-vrena-2025:v0.1.3
docker build -t vision-vrena-2025:v0.1.3 .

# 运行docker-compose.yml文件以镜像构造三个用dockernetwork链接可互相通信的容器
docker-compose up

关闭容器
docker-compose down 
#备赛期间可以不使用docker-compose方式运行容器，直接运行单个容器，建立网络链接。
```

# 二、 文件结构

```
├── README.md                # 项目说明文档
├── install/                  # 编译安装目录
├── src/                    # 例程源代码
├── results/                  # 得分结果文件
└── docs/                    # 完整文档
```


# 三、文档导航
**[launch文件使用说明](doc/launchsetting.md)**  
launch文件参数说明及使用方法

**[目标发送要求](doc/objectpublic.md)**  
目标边缘点信息发送要求

**[消息包说明](doc/objectmsg.md)**

消息包结构及说明

**[话题与服务消息说明](doc/Topic.md)**

 话题与服务消息说明  


**最后更新时间**：2025年9月30日  
**当前版本**：v1.0.0


