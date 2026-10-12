import numpy as np
from scipy.spatial.transform import Rotation as R

# Link lengths in meters (from the course homework)
l0 = 0.061     # base to servo 1
l1 = 0.0435    # servo 1 to servo 2
l2 = 0.08285   # servo 2 to servo 3
l3 = 0.08285   # servo 3 to servo 4
l4 = 0.07385   # servo 4 to servo 5
l5 = 0.05457   # servo 5 to gripper

# Unit vectors
ex = np.array([1, 0, 0])
ey = np.array([0, 1, 0])
ez = np.array([0, 0, 1])


# Rotations about the x, y, z axes, angle in degrees
def rotx(theta):
    return R.from_euler('x', theta, degrees=True)

def roty(theta):
    return R.from_euler('y', theta, degrees=True)

def rotz(theta):
    return R.from_euler('z', theta, degrees=True)


def fk_Dofbot(q):
    """
    q: 5 joint angles in degrees [q1, q2, q3, q4, q5]
       (these are the same numbers you send to the servos)
    Returns:
       Rot: rotation of the gripper relative to the base (scipy Rotation)
       Pot: position of the gripper tip relative to the base, in meters
    """
    # Rotation from each frame to the next
    R01 = rotz(q[0])     # base spin
    R12 = roty(-q[1])    # shoulder
    R23 = roty(-q[2])    # elbow
    R34 = roty(-q[3])    # wrist pitch
    R45 = rotx(-q[4])    # wrist roll
    R5T = roty(0)        # tool frame = frame 5

    # Distance from each frame to the next (in that frame, at q = 0)
    P01 = (l0 + l1) * ez
    P12 = np.zeros(3)
    P23 = l2 * ex
    P34 = -l3 * ez
    P45 = np.zeros(3)
    P5T = -(l4 + l5) * ex

    # Orientation: chain all the rotations together
    Rot = R01 * R12 * R23 * R34 * R45 * R5T

    # Position: start at the tip and work back to the base,
    # rotating each piece into the frame before it and adding
    Pot = P01 + R01.apply(P12 + R12.apply(P23 + R23.apply(P34 + R34.apply(P45 + R45.apply(P5T)))))

    return Rot, Pot


def pitch_from_vertical(Rot):
    # The gripper points along the tool frame's -x axis.
    # This gives the angle between that direction and straight up, in degrees.
    pointing = -Rot.as_matrix()[:, 0]
    return np.degrees(np.arccos(pointing[2]))
