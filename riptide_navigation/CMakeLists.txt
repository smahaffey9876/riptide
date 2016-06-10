cmake_minimum_required(VERSION 2.8.3)
project(riptide_navigation)

find_package(catkin REQUIRED COMPONENTS
  control_toolbox message_filters
  riptide_msgs sensor_msgs geometry_msgs
  tf
)
find_package(Ceres REQUIRED)

generate_dynamic_reconfigure_options(cfg/pid.cfg)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS})
###
add_executable(thrust_cal src/thrust_cal.cpp)
add_executable(thrust_mapper src/thrust_mapper.cpp)

add_executable(position_controller src/position_controller.cpp)
add_executable(joy_set src/joy_set.cpp)
add_executable(attitude_controller src/attitude_controller.cpp)
#add_executable(pid_pitch src/pid_pitch.cpp)
#add_executable(pid_yaw src/pid_yaw.cpp)
add_executable(my_tf_listener src/tf_listener.cpp)

add_dependencies(thrust_cal riptide_msgs_gencpp)
add_dependencies(thrust_mapper riptide_msgs_gencpp)

add_dependencies(position_controller riptide_navigation_gencfg)
add_dependencies(attitude_controller riptide_navigaon_gencfg)
#add_dependencies(pid_pitch riptide_navigation_gencfg)
#add_dependencies(pid_yaw riptide_navigation_gencfg)

target_link_libraries(thrust_cal ${catkin_LIBRARIES})
target_link_libraries(thrust_mapper ${catkin_LIBRARIES} ${CERES_LIBRARIES})

target_link_libraries(position_controller ${catkin_LIBRARIES})
target_link_libraries(joy_set ${catkin_LIBRARIES})
target_link_libraries(attitude_controller ${catkin_LIBRARIES})
#target_link_libraries(pid_pitch ${catkin_LIBRARIES})
#target_link_libraries(pid_yaw ${catkin_LIBRARIES})
target_link_libraries(my_tf_listener ${catkin_LIBRARIES})
