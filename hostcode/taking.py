import roboticstoolbox as rtb
import spatialmath as sm
import numpy as np
from swift import Swift

hight = 0.308

robot = rtb.models.MEPhI_ARM()
robot.q = robot.qz

env = Swift()
env.launch(realtime=True)
env.add(robot)


# def initQR():


def SolDimArray(tvec, trans_matrix, const_orient):
    # sol = SolDim(list(map(lambda x: round(x, ndigits = 4),tvec)))
    sol = SolDim(tvec, trans_matrix, const_orient)
    robot.q = sol
    env.step()
    for i in range(len(sol)):
        sol[i] = round((sol[i] / np.pi * 180), ndigits=1)
    return sol


def TrajFromQToPoint(tvec, trans_matrix, const_orient):
    sol = SolDim(tvec, trans_matrix, const_orient)
    traj = rtb.tools.trajectory.jtraj(robot.q, sol, 25)
    return traj


def SolDim(tvec, trans_matrix, const_orient):
    if (const_orient == True):
        new_y_axis_orientation = [-i for i in np.array(trans_matrix[0]).transpose()[1]]
        new_y_axis_orientation[2] = 0
        Tep = robot.fkine(robot.qr) * sm.SE3.Trans(tvec[0], tvec[1], tvec[2]) * sm.SE3.OA(new_y_axis_orientation,
                                                                                          [0, 0, 1])
    else:
        Tep = robot.fkine(robot.qr) * sm.SE3.Trans(tvec[0], tvec[1], tvec[2]) * sm.SE3.OA(
            [-i for i in np.array(trans_matrix[0]).transpose()[1]],
            [-i for i in np.array(trans_matrix[0]).transpose()[2]])

    sol = robot.ik_LM(Tep)

    if (chekTaskPresessing(tvec, sol[0])):
        return sol[0]
    else:
        # print("InvTask solved incorrectly")
        return robot.q


def chekTaskPresessing(tvec, sol):
    tvec_env = [0, 0, 0]

    # print((robot.fkine(sol).A)[2][3],(robot.fkine(robot.qr).A)[2][3])

    # СЛЕДИТЬ ЗА НАПРАВЛЕНИЕМ ОСЕЙ БЛЕД!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    for i in range(3):
        if i == 0:
            tvec_env[i] = ((robot.fkine(sol).A)[i][3] - (robot.fkine(robot.qr).A)[i][3])
        else:
            tvec_env[i] = ((robot.fkine(sol).A)[i][3] - (robot.fkine(robot.qr).A)[i][3]) * -1

    if (abs(tvec[2] - tvec_env[2]) + abs(tvec[0] - tvec_env[0]) + abs(tvec[1] - tvec_env[1]) < 0.005):
        return 1
    else:
        return 0