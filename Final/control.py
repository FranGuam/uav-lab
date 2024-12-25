import time

import cv2
import rospy

from tello import Tello, TelloROS


thresholds = {
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

def test_base_class():
    drone = Tello()
    drone.send_command("sdk?")
    drone.send_command("sn?")
    drone.send_command("battery?")
    drone.send_command("wifi?")
    drone.send_command("takeoff", 0)
    print("State: ", drone.get_state())
    img = cv2.cvtColor(drone.get_image(), cv2.COLOR_RGB2BGR)
    cv2.imshow("tello_image", img)
    drone.send_command("land", 0)
    cv2.waitKey(0)

def test_ros_class():
    rospy.init_node('tello', anonymous=True)
    drone = TelloROS()
    drone.send_command("sdk?")
    drone.send_command("sn?")
    drone.send_command("battery?")
    drone.send_command("wifi?")
    drone.send_command("takeoff", 0)
    print("State: ", drone.get_state())
    img = cv2.cvtColor(drone.get_image(), cv2.COLOR_RGB2BGR)
    cv2.imshow("tello_image", img)
    drone.send_command("land", 0)
    cv2.waitKey(0)

def test_color_detection():
    rospy.init_node('tello', anonymous=True)
    drone = TelloROS()
    time.sleep(1)
    img = cv2.cvtColor(drone.get_image(), cv2.COLOR_RGB2BGR)
    cv2.imshow("img", img)
    cv2.imwrite("test.png", img)
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    for color in thresholds:
        threshold = thresholds[color]
        mask = cv2.inRange(img_hsv, (threshold['H'][0], threshold['S'][0], threshold['V'][0]), (threshold['H'][1], threshold['S'][1], threshold['V'][1]))
        cv2.imshow('mask_'+color, mask)
    cv2.waitKey(0)

def go(drone, x, y, z, mid, speed=100):
    drone.send_command("go %d %d %d %d m%d" % (x, y, z, speed, mid), 0)

def test_go():
    rospy.init_node('tello', anonymous=True)
    drone = TelloROS()
    drone.send_command("mon")
    drone.send_command("takeoff", 0)
    while drone.get_state()['mid'] == -1:
        time.sleep(0.1)
    mid = drone.get_state()['mid']
    go(drone, 50, 50, 50, mid)
    drone.send_command("land", 0)

if __name__ == '__main__':
    # test_base_class()
    # test_ros_class()
    # test_color_detection()
    test_go()
