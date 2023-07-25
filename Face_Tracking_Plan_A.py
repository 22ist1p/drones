import cv2
import numpy as np
from djitellopy import Tello, TelloSwarm
import sys
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


def trackFace(info, w, pid, pError):
    fbRange = [6200, 6800]

    area = info[1]
    x, y = info[0]
    fb = 0

    error = x - w // 2
    speed = pid[0] * error + pid[1] * (error - pError)
    speed = int(np.clip(speed, -100, 100))

    if area > fbRange[0] and area < fbRange[1]:
        fb = 0
    elif area > fbRange[1]:
        fb = -20
    elif area < fbRange[0] and area != 0:
        fb = 20
    if x == 0:
        speed = 0
        error = 0

    me.send_rc_control(0, fb, 0, (speed - 10))
    return error

tellos = []

for ip in sys.argv[1:]:
    state_port, vs_port = calculate_ports(ip)

    me = Tello(host=ip, vs_udp=vs_port, state_port=state_port)
    me.connect()
    me.LOGGER.setLevel(logging.WARN)
    
    me.set_network_ports(state_port, vs_port)

    print(me.get_battery())

    me.streamon()
    # me.set_video_fps(me.FPS_30)
    tellos.append(me)

def script(index, me):
    is_flying = False

    reader = me.get_frame_read()

    me.send_rc_control(0, 0, 15, 0)
    # time.sleep(5)

    w, h = 360, 240
    
    pid = [0.4, 0.4, 0]
    # pid = [0.6, 0.4, 0]
    pError = 0

    frame_count = 0
    while True:
        img = reader.frame
        img = cv2.resize(img, (w, h))
        img, info = findFace(img)
        pError = trackFace(info, w, pid, pError)
        # print("Center", info[0], "Area", info[1])
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        cv2.imshow(str(index), img)
        if not is_flying and frame_count > 10:
            me.takeoff()
            is_flying = True
        if cv2.waitKey(1) & 0xFF == ord("q"):
            me.land()
            break
        frame_count += 1

swarm = TelloSwarm(tellos)

swarm.parallel(script)

cv2.destroyAllWindows()
