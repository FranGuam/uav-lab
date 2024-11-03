# 环境配置

无，连接到无人机 WiFi 即可。

# 资源

- Robomaster SDK: [Read The Doc: RoboMaster Tello SDK](https://robomaster-dev.readthedocs.io/zh-cn/latest/python_sdk/beginner_drone.html)
- DJITelloPy: [GitHub: DJITelloPy](https://github.com/damiafuentes/DJITelloPy?tab=readme-ov-file)

# 备注

1. `tello.apk` 是Tello的安卓手机APP，可以操纵 Tello 并查看实时视频，首次使用 Tello 时需要先连接手机APP进行激活。
2. `Tello.read_state()` 返回的是 `list[str]`，示例：`['mid:-2;', 'x:-200;', 'y:-200;', 'z:-200;', 'mpry:0,0,0;', 'pitch:0;', 'roll:0;', 'yaw:0;']`。其中
    - `mid` 是检测到的挑战卡 ID，未打开挑战卡探测功能返回 -2、打开探测功能但未检测到挑战卡返回 -1
    - `x`,`y`,`z` 分别是相对于挑战卡的三轴坐标，单位为厘米，未检测到挑战卡返回 -200、打开探测功能但未检测到挑战卡返回 -100
    - `mpry` 是飞行器在挑战卡中的俯仰、偏航及横滚角度，与下面定义相同，若未检测到挑战卡，返回 0
    - `pitch` 是前后俯仰，正是抬头、负是低头，取值-90到90
    - `roll` 是左右翻滚，左高右低为正、右高左低为负，取值-180到180
    - `yaw` 是z轴旋转角度，左转减小、右转增大，取值-180到180
