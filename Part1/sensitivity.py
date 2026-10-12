import time
import numpy as np
from Arm_Lib import Arm_Device
from DofbotKinematics import fk_Dofbot

arm = Arm_Device()

base = [90, 120, 60, 60, 90]   # the arm configuration we study
step = 5                       # perturbation size in degrees


def move(q):
    arm.Arm_serial_servo_write6(*q, 90, 1500)
    time.sleep(2)


def measure():
    m = input("Measured x y z in meters (space separated): ")
    return np.array([float(v) for v in m.split()])


def tip(q):
    Rot, Pot = fk_Dofbot(q)
    return Pot


# --- Jacobian (position part), computed numerically from the FK ---
# Column i = how much the tip moves per radian of joint i
def jacobian(q):
    tiny = 0.01  # degrees
    p0 = tip(q)
    J = np.zeros((3, 5))
    for i in range(5):
        q2 = list(q)
        q2[i] += tiny
        J[:, i] = (tip(q2) - p0) / np.radians(tiny)
    return J


J = jacobian(base)
print("Jacobian at base config (m per rad):\n", np.round(J, 4))

# Measure the base pose once
move(base)
print("\nArm at base config", base)
p_base_measured = measure()
p_base_fk = tip(base)

# Nudge one joint at a time
for i in range(5):
    pert = list(base)
    pert[i] += step
    move(pert)
    print(f"\nJoint {i+1} moved +{step} deg:", pert)
    p_pert_measured = measure()

    measured_change = p_pert_measured - p_base_measured
    fk_change = tip(pert) - p_base_fk
    jac_change = J[:, i] * np.radians(step)   # dp = J * dq

    print("Measured change (m):", np.round(measured_change, 4))
    print("FK change       (m):", np.round(fk_change, 4))
    print("J*dq change     (m):", np.round(jac_change, 4))

move(base)
