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

# 安装脚本和其他资源：

# 安装依赖脚本
install(DIRECTORY common/
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/common
)

# 获取scripts下python脚本
file(GLOB PYTHON_SCRIPTS
  "scripts/rohand.py"
  "scripts/rohand_serial.py"
  "scripts/rohand_teleop.py"
)

#安装脚本
catkin_install_python(PROGRAMS
  ${PYTHON_SCRIPTS}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


