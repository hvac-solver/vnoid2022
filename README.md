## Build 
- mkdir -p CNOID_WS/src && cd CNOID_WS/src
- git clone https://github.com/choreonoid/choreonoid
- git clone https://github.com/choreonoid/choreonoid_ros
- cd choreonoid/ext
- git clone https://github.com/hvac-solver/vnoid2022
- git clone https://github.com/jbeder/yaml-cpp
- cd ../../..
- catkin build choreonoid --cmake-args -DCMAKE_BUILD_TYPE=Release

## Choreonoid simulation projects
- fast_walking_controller.cnoid : Walking control simulation for the shorttrack competition
- athletic_controller.cnoid : Walking control simulation for the athletic competition

## Configs of controllers
- fast_walking_controller.config.cnoid 
- athletic_controller.config.cnoid 