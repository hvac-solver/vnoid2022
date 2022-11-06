
# For pinocchio and etc.
set(ENV{PATH} /opt/openrobots/bin:ENV{PATH})
set(ENV{PKG_CONFIG_PATH} /opt/openrobots/lib/pkgconfig:ENV{PKG_CONFIG_PATH})
set(ENV{LD_LIBRARY_PATH} /opt/openrobots/lib:ENV{LD_LIBRARY_PATH})

# Build
if(NOT CMAKE_BUILD_TYPE MATCHES Release)
  message(
    STATUS
      "WARNING: CMAKE_BUILD_TYPE is NOT set to Release, which can decrease performance significantly."
  )
endif()

# find Eigen
find_package(Eigen3 REQUIRED)
# find robotoc
find_package(robotoc REQUIRED)
# find yaml-cpp
find_package(yaml-cpp REQUIRED)

# fast_walking_controller (for the shorttrack competition)
choreonoid_add_simple_controller(
  fast_walking_controller 
  src/fast_walking_controller.cpp
)
target_link_libraries(
  fast_walking_controller
  robotoc::robotoc 
  yaml-cpp::yaml-cpp
)
target_include_directories(
  fast_walking_controller
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include 
  ${ROBOTOC_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIR}
)

# athletic_controller (for the athletic competition)
choreonoid_add_simple_controller(
  athletic_controller
  src/stair_climbing_foot_step_planner.cpp
  src/mpc_jump_com_ref.cpp
  src/biped_jump_foot_step_planner.cpp
  src/mpc_biped_jump.cpp
  src/athletic_controller.cpp
)
target_link_libraries(
  athletic_controller
  robotoc::robotoc 
  yaml-cpp::yaml-cpp
)
target_include_directories(
  athletic_controller
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include 
  ${ROBOTOC_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIR}
)

install(DIRECTORY model DESTINATION ${CNOID_SHARE_SUBDIR})
install(DIRECTORY project DESTINATION ${CNOID_SHARE_SUBDIR})