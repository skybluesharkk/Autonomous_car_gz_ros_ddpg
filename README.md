# 🏎️ Autonomous_car with GAZEBO, ROS2, and DDPG

## 개요

- 공식문서를 참고한 기본적인 차량 구현
- 경로 안내와 충돌 회피 기능을 위한 world구현
- ROS2를 활용한 통신 구현
- Pytorch를 활용한 DDPG 알고리즘 구현

## 센서들

- LiDAR (range scan or ray-cast distances)
- RGB camera image (front-facing camera)
- IMU (angular velocity and linear acceleration)
- Odometry (estimated robot pose and velocity)

## dependencies

````bash
conda activate hanyang_robot
conda env export > environment.yml```
````
