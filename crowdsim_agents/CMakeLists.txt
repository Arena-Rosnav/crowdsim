cmake_minimum_required(VERSION 3.0.2)
project(crowdsim_agents)
find_package(catkin REQUIRED)

catkin_package()

catkin_python_setup()
catkin_install_python(
    PROGRAMS nodes/crowdsim_agents
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)