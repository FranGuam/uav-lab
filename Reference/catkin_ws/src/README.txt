# build
$ sudo apt-get install ros-noetic-hector-pose-estimation ros-noetic-hector-sensors-description ros-noetic-message-to-tf
$ catkin_make

# run
roslaunch hector_quadrotor_demo start_nics_world.launch # 加载gazebo环境
roslaunch hector_quadrotor_demo put_robot_in_world.launch   # 放置无人机
rosrun teleop_twist_keyboard teleop_twist_keyboard.py   # 键盘控制无人机
rosrun hector_quadrotor_demo control_demo.py # 指令控制无人机demo, 按照时间控制无人机飞行， 具体指令可以查看hector_quadrotor/hector_quadrotor_demo/scripts/control_demo.py文件

rosrun hector_quadrotor_demo control_demo2.py # control 2
