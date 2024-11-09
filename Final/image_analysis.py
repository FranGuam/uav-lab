import cv2

# img = cv2.imread('orange.png')
# cv2.imshow('orange', img)

# img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
# points = [
#     [521, 475], # center
#     [609, 386], # top right
#     [432, 392], # top left
#     [439, 569], # bottom left
#     [609, 574], # bottom right
#     [520, 389], # highlight
#     [497, 517], # dark side
# ]
# for point in points:
#     print(img_hsv[point[1], point[0]])

# threshold = {
#     'H': [11, 18],
#     'S': [150, 255],
#     'V': [70, 255],
# }
# mask = cv2.inRange(img_hsv, (threshold['H'][0], threshold['S'][0], threshold['V'][0]), (threshold['H'][1], threshold['S'][1], threshold['V'][1]))
# cv2.imshow('mask_orange', mask)

# img = cv2.imread('green.png')
# cv2.imshow('green', img)

# img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
# points = [
#     [525, 475], # center
#     [590, 400], # top right
#     [456, 411], # top left
#     [460, 544], # bottom left
#     [593, 548], # bottom right
#     [506, 389], # highlight
#     [611, 472], # dark side
#     [546, 493],
#     [507, 385],
#     [477, 537]
# ]
# for point in points:
#     print(img_hsv[point[1], point[0]])

# threshold = {
#     'H': [41, 48],
#     'S': [150, 255],
#     'V': [70, 255],
# }
# mask = cv2.inRange(img_hsv, (threshold['H'][0], threshold['S'][0], threshold['V'][0]), (threshold['H'][1], threshold['S'][1], threshold['V'][1]))
# cv2.imshow('mask_green', mask)

# img = cv2.imread('blue.png')
# cv2.imshow('blue', img)

# img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
# points = [
#     [547, 415], # center
#     [665, 326], # top right
#     [488, 327], # top left
#     [480, 507], # bottom left
#     [666, 516], # bottom right
#     [557, 334], # highlight
#     [467, 451], # dark side
#     [460, 376],
#     [476, 490],
#     [680, 347]
# ]
# for point in points:
#     print(img_hsv[point[1], point[0]])

# threshold = {
#     'H': [105, 110],
#     'S': [150, 255],
#     'V': [70, 255],
# }
# mask = cv2.inRange(img_hsv, (threshold['H'][0], threshold['S'][0], threshold['V'][0]), (threshold['H'][1], threshold['S'][1], threshold['V'][1]))
# cv2.imshow('mask_blue', mask)

img = cv2.imread('test.png')
img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
points = [
    [246, 426],
]
for point in points:
    print(img_hsv[point[1], point[0]])

cv2.waitKey(0)
