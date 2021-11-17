# carlike_robot_sim  
Ackermann 转向机构的ROS机器人建图导航与仿真  

参考 https://blog.csdn.net/qq_36754438/article/details/109125320  

## 准备工作  
### 1. 下载代码并安装必要功能包  

环境：  

Ubuntu18.04 + ros melodic  

预先安装以下功能包（若有遗漏则根据报错自行安装）   

sudo apt-get install ros-melodic-ackermann-msgs  
sudo apt-get install ros-melodic-navigation  
sudo apt-get install ros-melodic-openslam-gmapping  
sudo apt-get install ros-melodic-geographic-info  
sudo apt-get install ros-melodic-controller-manager  
sudo apt-get install ros-melodic-gazebo-ros-control  
sudo apt-get install ros-melodic-effort-controllers  
sudo apt-get install ros-melodic-joint-state-controller   
sudo apt-get install ros-melodic-position-controllers   
sudo apt-get install ros-melodic-teb-local-planner  

### 2. 编译  

注意 bringup/map/mymap.yaml 中的 image 路径需要根据本机进行修改

### 3. 阿克曼小车控制

```shell
~$ roslaunch bringup car_sim_gmapping.launch	# 启动 gazebo 和 rviz 环境
~$ rosrun racecar_control keyboard_teleop.py	# 启动键盘控制节点
```

### 4. Gmapping 建图

在上一条操作的基础上，首先在rviz中载入 bringup/rviz 目录下的 gmapping.rviz 配置文件，再启动 gmmaping 

```shell
~$ roslaunch bringup slam_gmapping.launch
```

### 5. SLAM

```shell
~$ roslaunch bringup move_base.launch	
```

之后在 rviz 中用 2D Nav Goal 指定目标点即可进行导航



