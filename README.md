# Humanoid Virtual Athletics Challenge 2022 
(https://ytazz.github.io/vnoid/)

https://user-images.githubusercontent.com/33686357/201346741-d634b8d7-6eb0-418b-9cd0-23744085fab6.mp4


https://user-images.githubusercontent.com/33686357/201346835-3bec8c74-da74-4d58-b12a-4f3f405c0892.mp4


## Build 
```
mkdir -p CNOID_WS/src && cd CNOID_WS/src
git clone https://github.com/choreonoid/choreonoid
git clone https://github.com/choreonoid/choreonoid_ros
cd choreonoid/ext
git clone https://github.com/hvac-solver/vnoid2022
git clone https://github.com/jbeder/yaml-cpp
cd ../../..
catkin build choreonoid --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Choreonoid simulation projects
- fast_walking_controller.cnoid : Walking control simulation for the shorttrack competition
- athletic_controller.cnoid : Walking control simulation for the athletic competition

## Configs of controllers
- fast_walking_controller.config.cnoid 
- athletic_controller.config.cnoid 
