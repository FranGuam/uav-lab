# 环境配置

无

# 启动命令

加载gazebo环境：

```bash
roslaunch hector_quadrotor_demo start_nics_world.launch
```

放置无人机：

```bash
roslaunch hector_quadrotor_demo put_robot_in_world.launch
```

启动键盘控制：

```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

启动脚本控制：

```bash
rosrun hector_quadrotor_demo control_fire.py
```

重置无人机位置：

```bash
rosrun hector_quadrotor_demo set_model_state.py quadrotor 1 1 0.2 1.5707963267948966
```
