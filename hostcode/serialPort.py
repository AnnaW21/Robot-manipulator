import serial.tools.list_ports
import taking as tk
from numpy import pi

def serialBegin(port = 5, baytrate = 115200):
    global ser
    port = f"COM{port}"  # Replace with the appropriate COM port name
    ser = serial.Serial(port, baytrate)

def serialSend(deg, speed):
    dataArray = list(map(lambda x : round((x / pi * 180), ndigits=1), deg))
    dataArray.extend(list(map(lambda x : round((x / pi * 180), ndigits=1), speed)))
    output_text = ""
    output_text = "/".join(str(i) for i in dataArray)

    ser.write(output_text.encode('raw_unicode_escape'))
    print(output_text)

def serialRead():
    flag = 0
    while(not flag):
        flag = ser.read().decode()
    print(flag)

def sendTraj(traj):

    for i in range(len((traj.q))):
        tk.robot.q = traj.q[i]
        serialSend(traj.q[i], traj.qd[i])
        serialRead()