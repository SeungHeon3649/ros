# ros - gazebo 시뮬레이션에서 turtlebot3 제어

**※ gazebo simulation을 이용하여 사용자가 목표점을 입력하면 로봇이 목표점으로 이동**  
**※ 동작 영상**  
https://www.youtube.com/watch?v=Ot3r9NmQzZc  

**1. gazebo 시뮬레이션 실행**  
$ roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch  

![empty_world](https://user-images.githubusercontent.com/94602281/193763873-bd85d37c-5e95-45c9-b56f-21872b8a1413.png)

  
**2. drive_base 노드 실행**  
$ rosrun drive_base_tutorial drive_base
