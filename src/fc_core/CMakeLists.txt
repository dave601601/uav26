cmake_minimum_required(VERSION 3.10)
project(fc_core C CXX)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_C_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wno-unused-parameter)
endif()

find_package(ament_cmake REQUIRED)

set(FC_CORE_SOURCES
  src/linalg.c
  src/controller.c
  src/filter.c
  src/planner.c
  src/imu_parse.c
  src/sbus_parse.c
  src/lidar_parse.c
  src/protocol.c
  src/mission_ctrl.c
)

add_library(fc_core SHARED ${FC_CORE_SOURCES})
target_include_directories(fc_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(fc_core PUBLIC m)

install(DIRECTORY include/
  DESTINATION include)

install(TARGETS fc_core
  EXPORT export_fc_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

ament_export_targets(export_fc_core HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(fc_core)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_fc_core
    test/test_allocation.cpp
    test/test_rate_pid.cpp
    test/test_atti_pid.cpp
    test/test_quat.cpp
    test/test_protocol.cpp
    test/test_mission_ctrl.cpp
  )
  if(TARGET test_fc_core)
    target_link_libraries(test_fc_core fc_core)
  endif()
endif()

ament_package()
