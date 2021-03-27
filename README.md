# BUG2 simulation
## 写在前面

* 本项目是**BUG2**算法在**stm32F407**上运行的可行性验证实验。程序运行在stm32上，仿真环境为ROS+gazebo，二者通过串口通信。  

![实物展示]()  
![仿真界面展示]()  

* BUG2算法是机器人运动规划的一个经典算法，更详细的介绍及更多的机器人运动规划算法可移步: http://www.cs.cmu.edu/~motionplanning/lecture/lecture.html


## 实验说明
#### 小车模型：
![小车模型](https://github.com/TejasPhutane/Differential-Drive-robot-ROS-gazebo-teleop/blob/main/rrc_2wheel_robot.png)

* 小车的正前，正左，正右安装有三个超声波传感器，传感器的最大探测距离为0.2m，最小探测距离为0.02m, 精度0.01m。
* 小车提供差速模型，无需单独控制轮子转速，只需提供前进速度**v**, 转弯速度 **w**(逆时针为正)，即可控制小车运动。
* 小车提供由轮速计估计的里程计信息，在仿真环境里可以认为是无误差的。

#### 串口通信：
* stm32运行BUG2算法，需要的信息有目标点位置，当前位置，三个传感器的数据，因此将接收帧编码成如下格式：(每帧32bytes)

float(4bytes)  | float(4bytes)| float(4bytes)| float(4bytes)| float(4bytes)|float(4bytes)| float(4bytes)| float(4bytes)    
------------- | -------------|---|---
goal_x| goal_y|cur_x|cur_y|cur_yaw|sonar_f|sonar_l|sonar_r  


* 计算结果为机器人的控制信息，即前进速度v，角速度w, 因此将发送帧编码成如下格式：（每帧8bytes）

float(4bytes)  | float(4bytes)  
-------------|---  
v | w  

#### 实验结果
![实验结果]()  

## 安装与部署
#### PC 端
* Ubuntu18.04
* ROS
* gazebo
1. 下载仿真环境所需模型
```
mkdir -p ~/bug2_sim_ws/src && cd ~/bug2_sim_ws/src
git clone https://github.com/TejasPhutane/Differential-Drive-robot-ROS-gazebo-teleop.git
git clone https://github.com/ros-teleop/teleop_twist_keyboard.git
cd ..
catkin_make
source ./devel/setup.bash
```
2. 将本项目src/obstacle.world 放到 Differential-Drive-robot-ROS-gazebo-teleop 项目的 world文件夹里，并修改lauch文件，使得程序运行时加载 本项目的世界地图。
3. 将本项目src/com_with_stm32.py 放到 Differential-Drive-robot-ROS-gazebo-teleop 项目的script文件夹下（如果没有，新建一个script/）。
4. catkin_make 重新编译。
#### stm32端
* stm32F407ZG
* st-link
* micro-USB
1. 将本项目src/bug2_algo.cpp 使用 arduino IDE 通过 st-link 烧录到板子上。
2. 通过micro-USB连接stm32与PC, 进行串口通信。

