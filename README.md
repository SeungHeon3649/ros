# SLAM

※ **slam_cartographer를 사용하여 로봇의 위치추정 및 주변환경에 대한 지도를 작성하는 과정**  
※ **동작 영상**  
https://www.youtube.com/watch?v=nXYEexqR8kc  


**1. 사용권한을 준다**  
$ sudo chmod a+rw /dev/ttyUSB*  

**2. 다이나믹셀 패키지 사용**  
$ roslaunch dynamixel_workbench_controllers dynamixel_controllers.launch  

**3. SLAM 런치파일 실행**  
$ roslaunch turtlebot3_slam  turtlebot3_slam.launch  

**4. rviz 실행**  
$ rviz  
![slam](https://user-images.githubusercontent.com/94602281/201286428-d29e17c9-ff79-4040-9901-d9a127732b0a.png)


**5. turtlebot3-teleop 런치파일 실행(로봇조종)**  
$ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch  
