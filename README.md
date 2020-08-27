# CBD 2020 AI Robot Code

~~~ 
该代码基于RoboMaster的RobotRTS. https://github.com/RoboMaster/RoboRTS
~~~ 
需要安装RobotRTS的相关依赖
~~~ 
sudo apt-get install -y ros-kinetic-opencv3             \
                        ros-kinetic-cv-bridge           \
                        ros-kinetic-image-transport     \
                        ros-kinetic-stage-ros           \
                        ros-kinetic-map-server          \
                        ros-kinetic-laser-geometry      \
                        ros-kinetic-interactive-markers \
                        ros-kinetic-tf                  \
                        ros-kinetic-pcl-*               \
                        ros-kinetic-libg2o              \
                        ros-kinetic-rplidar-ros         \
                        ros-kinetic-rviz                \
                        protobuf-compiler               \
                        libprotobuf-dev                 \
                        libsuitesparse-dev              \
                        libgoogle-glog-dev              
~~~ 

## 测试方法如下
~~~ 
# 安装Ros环境,请使用ubuntu 16.04 & ROS kinetic 版本
# git clone 代码
# 安装相关依赖⬆
# 编译代码 catkin_make
# 运行仿真环境
roslaunch roborts_bringup roborts_stage.launch
# 待续。。。
~~~ 

### WiKi: https://www.kancloud.cn/zhouws/robot-2020/1887623

