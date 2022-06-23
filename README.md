# turtlesim을 이용해 목표점으로 이동



**1. 작업을 위해 패키지를 만든다**  
$ roscreate-pkg drive_base_tutorial roscpp geometry_msgs  

**2. 코드를 작성하기 위해 디렉터리 이동**  
$ cd catkin_ws/src/drive_base_tutorial/src  

**3. src 폴더에 drive_base.cpp 생성 후 코드 복붙**

**4. CMakeLists.txt에 다음 줄 추가**  
rosbuild_add_executable(drive_base src/drive_base.cpp)  

**5. drive_base_tutorial 디렉토리에서 make 입력 또는 $ cd ~/catkin_ws && catkin_make 입력**  

**6. roscore 실행**  
$ roscore  

**7. ros 기본 패키지인 터틀심 노드 실행**  
$ rosrun turtlesim turtlesim_node  

![turtlesim](https://user-images.githubusercontent.com/94602281/175324991-e01f681c-f9c6-491f-9ee6-01b8b1a6880c.png)

https://www.youtube.com/watch?v=oIZxgC9VQF0  

※**gazebo simulation을 이용한 목표점 이동**  
https://www.youtube.com/watch?v=Ot3r9NmQzZc
