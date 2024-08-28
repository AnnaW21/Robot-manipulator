import roboticstoolbox as rtb
import spatialmath as sm
import spatialgeometry as sg
import swift

env = swift.Swift()
env.launch(realtime=True)

robot = rtb.models.Puma560()
robot.q = robot.qz
env.add(robot)

Tep1 = robot.fkine(robot.q) * sm.SE3.Trans(0, -0.1, -0.2) * sm.SE3.OA([0, 0, -1], [0, 1, 0])
sol1 = robot.ik_LM(Tep1)
goal_ax = sg.Axes(0.1, pose=Tep1)
env.add(goal_ax)
traj1 = rtb.tools.trajectory.jtraj(robot.qz, sol1[0], 100)
print(traj1.q)
for i in traj1.q:  # for each joint configuration on trajectory
    robot.q = i  # update the robot state
    env.step(0.01)  # update visualization

Tep2 = robot.fkine(robot.q) * sm.SE3.Trans(0.2, 0, 0) * sm.SE3.OA([0, 1, 0], [0, 0, 1])
goal_ax = sg.Axes(0.1, pose=Tep2)
env.add(goal_ax)

dt = 1
tacc = 0.5
qdmax = 0.1

Tc = rtb.ctraj(Tep1, Tep2, 25)  # Cartesian trajectory
cart_sol = robot.ikine_LM(Tc)
cart_traj = rtb.tools.trajectory.mstraj(cart_sol.q, dt, tacc, qdmax)
# print(cart_traj.q)

for i in cart_traj.q:
    # goal_ax = sg.Axes(0.1, pose=robot.fkine(i))
    # env.add(goal_ax)
    robot.q = i
    env.step(0.01)
