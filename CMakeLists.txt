
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

# fast_walking_controller
choreonoid_add_simple_controller(
  fast_walking_controller 
  src/fast_walking_controller.cpp
)
target_link_libraries(
  fast_walking_controller
  robotoc::robotoc 
)
target_include_directories(
  fast_walking_controller
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include 
  ${ROBOTOC_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIR}
)

install(DIRECTORY model DESTINATION ${CNOID_SHARE_SUBDIR})
install(DIRECTORY project DESTINATION ${CNOID_SHARE_SUBDIR})