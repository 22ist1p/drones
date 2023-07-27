import cv2
import numpy as np
from djitellopy import Tello, TelloSwarm
import sys
import math
import time
import logging


def calculate_ports(ip):
    "Первый - для STATE_UDP_PORT, второй - VS_UDP_PORT"
    id = int(ip.split('.')[-1])
    return 9000 + id * 10, 11111 + id * 10


def findFace(img):
    faceCascade = cv2.CascadeClassifier(
        "Resours/haarcascade_frontalface_default.xml")
    imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    faces = faceCascade.detectMultiScale(imgGray, 1.2, 8)

    myFaceListC = []
    myFaceListArea = []

    for (x, y, w, h) in faces:
        cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 2)
        cx = x + w // 2
        cy = y + h // 2
        area = w * h
        cv2.circle(img, (cx, cy), 5, (0, 255, 0), cv2.FILLED)
        myFaceListC.append([cx, cy])
        myFaceListArea.append(area)
    if len(myFaceListArea) != 0:
        i = myFaceListArea.index(max(myFaceListArea))
        return img, [myFaceListC[i], myFaceListArea[i]]
    else:
        return img, [[0, 0], 0]


def trackFace(info, frame_width, frame_height, pError):
    area = info[1]
    x, y = info[0]

    area_deadzone = 100
    area_target = 6900

    pid = [0.4, 0.4, 0]
    error = x - frame_width // 2
    speed = pid[0] * error + pid[1] * (error - pError)
    speed = int(np.clip(speed, -100, 100))

    area_relative_to_target = area - area_target

    forward_backward_velocity = int(
        max(0, math.sqrt(5 + abs(area_relative_to_target/8) - 5)))
    if area_relative_to_target > area_deadzone:
        forward_backward_velocity *= -1
    elif area_relative_to_target < -area_deadzone:
        pass
    else:
        forward_backward_velocity = 0

    y_deadzone = 25
    y_relative_to_center = y - frame_height // 2

    up_down_velocity = int(
        max(0, math.sqrt(10 + abs(y_relative_to_center*10) - 10)))
    if y_relative_to_center > y_deadzone:
        up_down_velocity *= -1
    elif y_relative_to_center < -y_deadzone:
        pass
    else:
        up_down_velocity = 0

    if x == 0 or y == 0:
        speed = 0
        error = 0
        up_down_velocity = 0
        forward_backward_velocity = 0

    me.send_rc_control(0, forward_backward_velocity, up_down_velocity, speed)
    return error


tellos = []

for ip in sys.argv[1:]:
    state_port, vs_port = calculate_ports(ip)

    me = Tello(host=ip, vs_udp=vs_port)
    me.connect()
    me.LOGGER.setLevel(logging.WARN)

    me.set_network_ports(state_port, vs_port)

    print(me.get_battery())

    me.streamon()
    me.set_video_bitrate(me.BITRATE_5MBPS)
    me.set_video_fps(me.FPS_30)
    me.set_video_resolution(me.RESOLUTION_480P)
    # me.set_video_fps(me.FPS_30)
    tellos.append(me)


def script(index, me):

    reader = me.get_frame_read()
    reader.frame

    me.takeoff()
    me.move_up(30)

    frame_width, frame_height = 648, 478

    # pid = [0.6, 0.4, 0]
    pError = 0

    frame_count = 0
    while True:
        if frame_count % 2:
            continue
        img = reader.frame
        # cv2.imshow(str(index+1), img)
        # img = cv2.resize(img, (frame_width, frame_height))
        img, info = findFace(img)
        pError = trackFace(info, frame_width, frame_height, pError)
        # print("Center", info[0], "Area", info[1])
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        cv2.imshow(str(index), img)
        frame_count += 1
        if cv2.waitKey(1) & 0xFF == ord("q"):
            cv2.destroyAllWindows()
            break


swarm = TelloSwarm(tellos)

swarm.sequential(script)

swarm.land()
