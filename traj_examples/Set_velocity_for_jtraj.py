import roboticstoolbox as rtb
import spatialmath as sm
import spatialgeometry as sg
import swift
import numpy as np
import time
import matplotlib.pyplot as plt

env = swift.Swift()
env.launch(realtime=True)

robot = rtb.models.MEPhI_ARM()
robot.q = robot.qr
env.add(robot)

time.sleep(1)

Tep1 = robot.fkine(robot.q) * sm.SE3.Trans(0.0, 0.0, 0.15) * sm.SE3.OA([0, 1, 0], [0, 0, 1])
sol = robot.ik_LM(Tep1)

goal_ax = sg.Axes(0.1, pose=Tep1)
env.add(goal_ax)

travel_time = 5
step = 50
step_time = travel_time / step
time_vec = np.arange(0, travel_time, step_time)
traj = rtb.tools.trajectory.jtraj(robot.q, sol[0], time_vec, qd0=[0.005, 0.005, 0.005, 0.005, 0.005, 0.005],
                                  qd1=[0.005, 0.005, 0.005, 0.005, 0.005, 0.005])
# print(traj.q)

# графики скоростей в осях
graphics = traj.qd.transpose()
for i in range(6):
    if i != 6:
        plt.plot(graphics[i] * 180 / np.pi, label=f"{i + 1}")
plt.legend()
plt.show()

# движение робота
for i in traj.q:
    robot.q = i
    env.step(step_time)
