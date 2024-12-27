import math
import time

import cv2
import rospy
from std_msgs.msg import String

from tello import Tello, TelloROS


hsv_thresholds = {
    'orange': {
        'H': [11, 18],
        'S': [120, 255],
        'V': [70, 255],
    },
    'green': {
        'H': [41, 48],
        'S': [120, 255],
        'V': [70, 255],
    },
    'blue': {
        'H': [105, 110],
        'S': [120, 255],
        'V': [70, 255],
    },
}

size_thresholds = {
    'orange': 50000,
    'green': 10000,
    'blue': 10000,
}

abbr = {
    'orange': 'R',
    'green': 'G',
    'blue': 'B',
}

def test_base_class():
    drone = Tello()
    drone.send_command_and_receive_response("sdk?")
    drone.send_command_and_receive_response("sn?")
    drone.send_command_and_receive_response("battery?")
    drone.send_command_and_receive_response("wifi?")
    drone.send_command_and_receive_response("takeoff", 20)
    print("State: ", drone.get_state())
    img = cv2.cvtColor(drone.get_image(), cv2.COLOR_RGB2BGR)
    cv2.imshow("tello_image", img)
    drone.send_command_and_receive_response("land", 20)
    cv2.waitKey(0)

def test_ros_class():
    rospy.init_node('tello', anonymous=True)
    drone = TelloROS()
    drone.send_command_and_receive_response("sdk?")
    drone.send_command_and_receive_response("sn?")
    drone.send_command_and_receive_response("battery?")
    drone.send_command_and_receive_response("wifi?")
    drone.send_command_and_receive_response("takeoff", 20)
    print("State: ", drone.get_state())
    img = cv2.cvtColor(drone.get_image(), cv2.COLOR_RGB2BGR)
    cv2.imshow("tello_image", img)
    drone.send_command_and_receive_response("land", 20)
    cv2.waitKey(0)

def test_state():
    rospy.init_node('tello', anonymous=True)
    drone = TelloROS()
    drone.send_command_and_receive_response("mon")
    while True:
        print("State: ", drone.get_state())

def test_color_detection():
    rospy.init_node('tello', anonymous=True)
    drone = TelloROS()
    time.sleep(1)
    img = cv2.cvtColor(drone.get_image(), cv2.COLOR_RGB2BGR)
    cv2.imshow("img", img)
    cv2.imwrite("test.png", img)
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    for color in hsv_thresholds:
        threshold = hsv_thresholds[color]
        mask = cv2.inRange(img_hsv, (threshold['H'][0], threshold['S'][0], threshold['V'][0]), (threshold['H'][1], threshold['S'][1], threshold['V'][1]))
        cv2.imshow('mask_'+color, mask)
    cv2.waitKey(0)

def color_detection(img, hsv_thresholds, size_thresholds, debug=False, debug_prefix=""):
    if debug:
        print("Detecting color for ", debug_prefix)
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        cv2.imwrite(debug_prefix + "_original.png", img)
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    else:
        img_hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    color_size = {}
    for color in hsv_thresholds:
        threshold = hsv_thresholds[color]
        mask = cv2.inRange(img_hsv, (threshold['H'][0], threshold['S'][0], threshold['V'][0]), (threshold['H'][1], threshold['S'][1], threshold['V'][1]))
        color_size[color] = cv2.countNonZero(mask)
        if debug:
            print(color, ":", color_size[color])
            cv2.imwrite(debug_prefix + "_mask_" + color + ".png", mask)
    for color in color_size:
        color_size[color] -= size_thresholds[color]
    max_color = max(color_size, key=color_size.get)
    if color_size[max_color] > 0:
        return max_color
    else:
        return None

def go(drone, x, y, z, mid, speed=100):
    drone.send_command_and_receive_response("go %d %d %d %d m%d" % (x, y, z, speed, mid), 20)

def test_go():
    rospy.init_node('tello', anonymous=True)
    drone = TelloROS()
    drone.send_command_and_receive_response("mon")
    drone.send_command_and_receive_response("takeoff", 20)
    while drone.get_state()['mid'] == -1:
        time.sleep(0.1)
    mid = drone.get_state()['mid']
    go(drone, 50, 50, 150, mid)
    drone.send_command_and_receive_response("land", 20)

def curve(drone, x1, y1, z1, x2, y2, z2, mid, speed=60):
    drone.send_command_and_receive_response("curve %d %d %d %d %d %d %d m%d" % (x1, y1, z1, x2, y2, z2, speed, mid), 20)

def test_curve():
    rospy.init_node('tello', anonymous=True)
    drone = TelloROS()
    drone.send_command_and_receive_response("mon")
    drone.send_command_and_receive_response("takeoff", 20)
    while drone.get_state()['mid'] == -1:
        time.sleep(0.1)
    mid = drone.get_state()['mid']
    go(drone, -50, -50, 150, mid)
    curve(drone, 50, -50, 150, 50, 50, 150, mid)
    drone.send_command_and_receive_response("land", 20)

def trim_angle(deg):
    if deg > 180:
        deg -= 360
    elif deg < -180:
        deg += 360
    return deg

def turn(drone, yaw, use_mpry=False, yaw_0=0):
    if use_mpry:
        current = drone.get_state()['mpry'][1]
    else:
        current = trim_angle(-(drone.get_state()['yaw'] - yaw_0))
    delta = trim_angle(yaw - current)
    if delta > 0:
        drone.send_command_and_receive_response("ccw %d" % delta, 10)
    else:
        drone.send_command_and_receive_response("cw %d" % -delta, 10)

def turn_to(drone, x, y, use_mpry=False, yaw_0=0):
    current_x = drone.get_state()['x']
    current_y = drone.get_state()['y']
    delta_x = x - current_x
    delta_y = y - current_y
    yaw = math.degrees(math.atan2(delta_y, delta_x))
    turn(drone, yaw, use_mpry, yaw_0)

def test_turn():
    rospy.init_node('tello', anonymous=True)
    drone = TelloROS()
    drone.send_command_and_receive_response("mon")
    drone.send_command_and_receive_response("takeoff", 20)
    while drone.get_state()['mid'] == -1:
        time.sleep(0.1)
    yaw_0 = trim_angle(drone.get_state()['yaw'] + drone.get_state()['mpry'][1])
    print("Yaw 0: ", yaw_0)
    turn(drone, 0, False, yaw_0)
    time.sleep(1)
    turn(drone, 180, False, yaw_0)
    time.sleep(1)
    turn(drone, 90, False, yaw_0)
    time.sleep(1)
    turn(drone, -180, False, yaw_0)
    time.sleep(1)
    turn(drone, 90, False, yaw_0)
    time.sleep(1)
    turn_to(drone, 0, 0, False, yaw_0)
    time.sleep(1)
    drone.send_command_and_receive_response("land", 20)

def rc(drone, roll, pitch, throttle, yaw):
    drone.send_command("rc %d %d %d %d" % (roll, pitch, throttle, yaw))

def test_rc():
    rospy.init_node('tello', anonymous=True)
    drone = TelloROS()
    drone.send_command_and_receive_response("mon")
    drone.send_command_and_receive_response("takeoff", 20)
    while drone.get_state()['mid'] == -1:
        time.sleep(0.1)
    mid = drone.get_state()['mid']
    yaw_0 = trim_angle(drone.get_state()['yaw'] + drone.get_state()['mpry'][1])
    go(drone, -50, -50, 150, mid)
    turn(drone, 40, False, yaw_0)
    rc(drone, -28, 0, 0, 50)
    time.sleep(8)
    rc(drone, 0, 0, 0, 0)
    drone.send_command_and_receive_response("land", 20)

def test_judge():
    rospy.init_node('tello', anonymous=True)
    judge_pub = rospy.Publisher('judge', String, queue_size=1)
    drone = TelloROS()
    drone.send_command_and_receive_response("takeoff", 20)
    judge_pub.publish("RGB")
    drone.send_command_and_receive_response("land", 20)

def stage_1():
    def task_1(drone, mid, yaw_0):
        go(drone, -60, -175, 135, mid)
        turn(drone, 0, False, yaw_0)
        color = color_detection(drone.get_image(), hsv_thresholds, size_thresholds, debug=True, debug_prefix="task_1")
        if color:
            return abbr[color]
        else:
            return 'R'

    def task_2_circle(drone, mid, yaw_0):
        go(drone, -50, -50, 130, mid)
        turn(drone, 40, False, yaw_0)
        rc(drone, -28, 0, 0, 50)
        ans = ''
        start = time.time()
        while time.time() - start < 8:
            color = color_detection(drone.get_image(), hsv_thresholds, size_thresholds)
            if color:
                ans = abbr[color]
                break
        if time.time() - start < 2:
            go(drone, -50, 50, 140, mid)
        else:
            rc(drone, 0, 0, 0, 0)
        if ans == '':
            ans = 'R'
        return ans

    def task_2_square(drone, mid, yaw_0):
        go(drone, -50, -50, 130, mid)
        turn(drone, 45, False, yaw_0)
        color = color_detection(drone.get_image(), hsv_thresholds, size_thresholds, debug=True, debug_prefix="task_2_1")
        if color:
            go(drone, -50, 50, 140, mid)
            return abbr[color]
        go(drone, -70, 0, 130, mid)
        turn(drone, 0, False, yaw_0)
        color = color_detection(drone.get_image(), hsv_thresholds, size_thresholds, debug=True, debug_prefix="task_2_2")
        if color:
            go(drone, -50, 50, 140, mid)
            return abbr[color]
        go(drone, -50, 50, 130, mid)
        turn(drone, -45, False, yaw_0)
        color = color_detection(drone.get_image(), hsv_thresholds, size_thresholds, debug=True, debug_prefix="task_2_3")
        if color:
            return abbr[color]
        go(drone, 0, 70, 130, mid)
        turn(drone, -90, False, yaw_0)
        color = color_detection(drone.get_image(), hsv_thresholds, size_thresholds, debug=True, debug_prefix="task_2_4")
        if color:
            return abbr[color]
        go(drone, 50, 50, 130, mid)
        turn(drone, -135, False, yaw_0)
        color = color_detection(drone.get_image(), hsv_thresholds, size_thresholds, debug=True, debug_prefix="task_2_5")
        if color:
            return abbr[color]
        go(drone, 70, 0, 130, mid)
        turn(drone, 180, False, yaw_0)
        color = color_detection(drone.get_image(), hsv_thresholds, size_thresholds, debug=True, debug_prefix="task_2_6")
        if color:
            return abbr[color]
        go(drone, 50, -50, 130, mid)
        turn(drone, 135, False, yaw_0)
        color = color_detection(drone.get_image(), hsv_thresholds, size_thresholds, debug=True, debug_prefix="task_2_7")
        if color:
            return abbr[color]
        go(drone, 0, -70, 130, mid)
        turn(drone, 90, False, yaw_0)
        color = color_detection(drone.get_image(), hsv_thresholds, size_thresholds, debug=True, debug_prefix="task_2_8")
        go(drone, 50, -50, 140, mid)
        if color:
            return abbr[color]
        return 'R'

    def task_3(drone, mid, yaw_0):
        go(drone, 70, 185, 140, mid)
        turn(drone, 180, False, yaw_0)
        color = color_detection(drone.get_image(), hsv_thresholds, size_thresholds, debug=True, debug_prefix="task_3")
        if color:
            return abbr[color]
        else:
            return 'R'

    rospy.init_node('tello', anonymous=True)
    judge_pub = rospy.Publisher('judge', String, queue_size=1)
    drone = TelloROS()
    drone.send_command_and_receive_response("mon")
    response = drone.send_command_and_receive_response("takeoff", 20)
    if response != 'ok':
        return
    while drone.get_state()['mid'] < 0:
        time.sleep(0.1)
    mid = drone.get_state()['mid']
    yaw_0 = trim_angle(drone.get_state()['yaw'] + drone.get_state()['mpry'][1])

    task_1_ans = task_1(drone, mid, yaw_0)
    task_2_ans = task_2_square(drone, mid, yaw_0)
    task_3_ans = task_3(drone, mid, yaw_0)
    judge_pub.publish(task_1_ans + task_2_ans + task_3_ans)
    go(drone, 210, 210, 30, mid)
    drone.send_command_and_receive_response("land", 20)


if __name__ == '__main__':
    # test_base_class()
    # test_ros_class()
    # test_state()
    # test_color_detection()
    # test_go()
    # test_curve()
    # test_turn()
    # test_rc()
    # test_judge()
    stage_1()
