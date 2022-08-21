cmake_minimum_required(VERSION 3.5)
project(ros_kvh1750)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)

ament_auto_add_library(kvh1750
  src/iomodule.cpp
  src/types.cpp
  src/imu.cpp
  src/tov_file.cpp
  src/bit_message.cpp)

ament_auto_add_library(driver_component
  src/driver_component.cpp)
target_link_libraries(driver_component kvh_1750)

#catkin_add_gtest(kvh1750-test test/utest.cpp)
#if(TARGET ${PROJECT_NAME}-test)
#  target_link_libraries(kvh1750-test kvh1750 ${catkin_LIBRARIES})
#endif()

##
#add_executable(kvh_1750_imu src/node.cpp)
#target_link_libraries(kvh_1750_imu kvh1750 ${catkin_LIBRARIES})

#############
## Install ##
#############
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_auto_package()
