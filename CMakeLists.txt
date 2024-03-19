cmake_minimum_required(VERSION 2.8.3)
project(tocabi_avatar)

add_compile_options(-std=c++17 -O3)

find_package(catkin REQUIRED
roscpp
tocabi_lib
tocabi_msgs
eigen_conversions
)

find_package(casadi REQUIRED) # KW add (2024-03-13)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS tocabi_lib tocabi_msgs
)

# set casadi include directory
set(LIBRARY_DIRS /usr/local/lib)  # KW add (2024-03-13)
set(CASADI_INCLUDE_DIR /usr/local/include/casadi) # KW add (2024-03-13)

###########
## Build ##
###########
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${CASADI_INCLUDE_DIR} # KW add (2024-03-13)
)

add_library(${PROJECT_NAME}
  src/avatar.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${LIBRARY_DIRS}/libcasadi.so  # KW add (2024-03-13)
)

# target_link_libraries(${PROJECT_NAME} PUBLIC lexls::lexls)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)