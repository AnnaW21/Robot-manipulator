from tkinter.constants import ROUND

import serial
from numpy.f2py.f90mod_rules import fgetdims2

import traj_planning as tk
import time

changed_q = [[0] * 6, [0] * 6]


def serialBegin(port=5, baytrate=115200):
    global ser
    port = f"COM{port}"  # Replace with the appropriate COM port name
    ser = serial.Serial(port, baytrate)
    print("wew")
    time.sleep(1)  # чтобы esp32 успела инициализироваться


def serialSend(deg, speed):
    for i in range(6):
        if(abs(speed[i]) < 0.02):
            deg[i] = tk.robot.q[i]
            changed_q[0][i] = 1
            changed_q[1][i] = tk.robot.q[i]
            speed[i] = 0.05
            # print(100*"-", f"ось {i}   ", speed[i])
        #
        # elif (changed_q[0][i]):
        #     speed[i] = (deg[i] - changed_q[1][i]) / tk.step_time
        #     # speed[i] = 0.02
        #     changed_q[0][i] = 0

    tk.robot.q = deg

    dataArray = list(map(lambda x: round(x * 180 / 3.14159, ndigits=4), deg))
    dataArray.extend(list(map(lambda x: round(x * 180 / 3.14159, ndigits=4), speed)))

    for i in [1, 0]:
        dataArray[i] *= -1

    # dataArray[5] *= 0

    output_text = ""
    output_text = "/".join(str(i) for i in dataArray)
    ser.write(bytes(output_text, 'utf-8'))
    print(bytes(output_text, 'utf-8'))


def serialRead(printFlag = True):
    flag = '0'
    while (flag == '0'):
        flag = ser.read()
        if(printFlag):
            print(flag)


def sendTraj(traj):
    traj.q[-2] = traj.q[-1]
    for i in range(len(traj.q)):
        print(i, end=' ')
        serialSend(traj.q[i], traj.qd[i])
        serialRead()
    ser.write(bytes("101a", 'utf-8'))
    serialRead()
    for i in range(len(traj.q)-1):
        tk.robot.q = traj.q[i]
        tk.env.step(tk.step_time)
    serialRead()