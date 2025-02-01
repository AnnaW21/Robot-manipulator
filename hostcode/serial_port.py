from tkinter.constants import ROUND

import serial

import traj_planning as tk
import time

from numpy import linspace


changed_q = [[0] * 6, [0] * 6]


def serialBegin(port=5, baytrate=115200):
    global ser
    port = f"COM{port}"  # Replace with the appropriate COM port name
    ser = serial.Serial(port, baytrate)
    print("embedded devise is connected!")
    time.sleep(1)  # чтобы esp32 успела инициализироваться


def serialSend(deg, speed, angle):
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
    tk.robot.ga = angle

    dataArray = list(map(lambda x: round(x * 180 / 3.14159, ndigits=4), deg))
    dataArray.extend(list(map(lambda x: round(x * 180 / 3.14159, ndigits=4), speed)))
    dataArray.append(round(angle, ndigits= 4))

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


def sendTraj(traj, servoStatesArr = None):

    """"
    Данная функция принимает на вход объект traj и массив servoStatesArr и отправляет их на подключаемое устройство в нужном формате

    traj - объект из библиотеки robotics-toolbox
    servoStatesArr - массив из объектов вида (perсent, degree), где perсent - процент выполнения траектории, на котором сервопривод будет растворен на нужный угол, degree - угол,
    на который будет растворен сервопривод, в градусах

    промежуточные состояния сервопривода достраиваются линейной интерполяцией

    """
    servoTraj = [tk.robot.ga]*len(traj.q)
    
    if servoStatesArr is not None:
        servoSecialPoints = [0]
        servoSecialAngles = [tk.robot.ga]

        for i in servoStatesArr:
            servoSecialPoints.append(int(i[0]*len(traj.q)))
            servoSecialAngles.append(i[1])

        for i in range(len(servoSecialPoints) - 1):
            servoTraj[servoSecialPoints[i] : servoSecialPoints[i+1]] = linspace(servoSecialAngles[i], servoSecialAngles[i+1], servoSecialPoints[i+1] - servoSecialPoints[i])


    traj.q[-2] = traj.q[-1]
    
    for i in range(len(traj.q)):
        print(i, end=' ')
        serialSend(traj.q[i], traj.qd[i], servoTraj[i])
        serialRead()
    
    ser.write(bytes("101a", 'utf-8'))
    serialRead()
    
    for i in range(len(traj.q)-1):
        tk.robot.q = traj.q[i]
        tk.env.step(tk.step_time)
    
    serialRead()