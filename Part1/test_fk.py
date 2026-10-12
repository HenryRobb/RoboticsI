import time
import numpy as np
from Arm_Lib import Arm_Device
from DofbotKinematics import fk_Dofbot, pitch_from_vertical

arm = Arm_Device()

# Five joint angle sets [q1, q2, q3, q4, q5] in degrees
test_sets = [
    [90,  90,  90,  90, 90],
    [90, 120,  90,  90, 90],
    [45, 110,  70,  60, 90],
    [135, 100, 120, 100, 90],
    [90, 135,  45,  45, 90],
]

for q in test_sets:
    # Move the arm (6th value is the gripper, 2000 = take 2 seconds)
    arm.Arm_serial_servo_write6(*q, 90, 2000)
    time.sleep(2.5)

    Rot, Pot = fk_Dofbot(q)
    pitch = pitch_from_vertical(Rot)

    print("\nAngles:", q)
    print("Predicted rotation matrix:\n", np.round(Rot.as_matrix(), 3))
    print("Predicted x y z (m):", np.round(Pot, 4))
    print("Predicted pitch from vertical (deg):", round(pitch, 1))

    # Measure the gripper tip with a ruler and type it in
    m = input("Measured x y z in meters (space separated): ")
    measured = np.array([float(v) for v in m.split()])
    m_pitch = float(input("Measured pitch from vertical (deg): "))

    print("Position error (m):", np.round(measured - Pot, 4))
    print("Error distance (m):", round(np.linalg.norm(measured - Pot), 4))
    print("Pitch error (deg):", round(m_pitch - pitch, 1))
