cmake_minimum_required(VERSION 3.0.2)
project(rohand)

# 查找 catkin 和其他依赖
find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
)

# 声明一个 catkin 包
catkin_package(
  CATKIN_DEPENDS rospy sensor_msgs std_msgs  # 这里列出包的依赖
)

# 启用配置文件
catkin_python_setup()

# 获取scripts下python脚本
file(GLOB PYTHON_SCRIPTS
  "scripts/roh_a001_node/*.py"
  "scripts/roh_ap001_node/*.py"
  "scripts/roh_lites001_node/*.py"
)

#安装脚本
catkin_install_python(PROGRAMS
  ${PYTHON_SCRIPTS}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


