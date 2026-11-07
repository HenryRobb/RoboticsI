"""Camera pose tester for the DOFBot.

Type joint angles (degrees) to move the arm, watch the live camera feed, and
save the poses you like for each view (dealer cards, my cards, my chips, bet).

Run on the Jetson (with a display / VNC):  python3 camera_pose_tester.py

Commands at the prompt:
  90 90 90 90 90          set joints 1-5 (gripper keeps its current value)
  90 90 90 90 90 180      set joints 1-6
  j3 45                   set a single joint
  save dealer             save current pose under a name (writes poses.json)
  go dealer               move to a saved pose
  list                    show saved poses
  snap                    save a screenshot of the current frame
  q                       quit
"""
import json
import os
import threading
import time

import cv2
from Arm_Lib import Arm_Device

POSES_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "poses.json")
MOVE_TIME_MS = 1500
CAMERA_INDEX = 0

arm = Arm_Device()
joints = [90, 90, 90, 90, 90, 90]  # 6th = gripper
running = True


def move(q):
    q = [max(0, min(180, int(round(a)))) for a in q]
    joints[:] = q
    arm.Arm_serial_servo_write6(*q, MOVE_TIME_MS)
    time.sleep(MOVE_TIME_MS / 1000 + 0.2)


def load_poses():
    if os.path.exists(POSES_FILE):
        with open(POSES_FILE) as f:
            return json.load(f)
    return {}


def save_poses(poses):
    with open(POSES_FILE, "w") as f:
        json.dump(poses, f, indent=2)


def camera_loop():
    global running
    cap = cv2.VideoCapture(CAMERA_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    while running:
        ok, frame = cap.read()
        if not ok:
            time.sleep(0.05)
            continue
        h, w = frame.shape[:2]
        # crosshair + current angles overlay
        cv2.line(frame, (w // 2 - 15, h // 2), (w // 2 + 15, h // 2), (0, 255, 0), 1)
        cv2.line(frame, (w // 2, h // 2 - 15), (w // 2, h // 2 + 15), (0, 255, 0), 1)
        cv2.putText(frame, "q: " + " ".join(str(a) for a in joints), (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        camera_loop.latest = frame.copy()
        cv2.imshow("DOFBot camera", frame)
        if cv2.waitKey(1) & 0xFF == 27:  # Esc in window also quits
            running = False
    cap.release()
    cv2.destroyAllWindows()


camera_loop.latest = None


def main():
    global running
    poses = load_poses()
    threading.Thread(target=camera_loop, daemon=True).start()
    move(joints)
    print(__doc__)

    while running:
        try:
            cmd = input("pose> ").strip()
        except (EOFError, KeyboardInterrupt):
            break
        if not cmd:
            continue
        parts = cmd.split()
        try:
            if parts[0] in ("q", "quit", "exit"):
                break
            elif parts[0] == "save" and len(parts) == 2:
                poses[parts[1]] = list(joints)
                save_poses(poses)
                print("saved %s = %s" % (parts[1], joints))
            elif parts[0] == "go" and len(parts) == 2:
                if parts[1] in poses:
                    move(poses[parts[1]])
                else:
                    print("no pose named", parts[1])
            elif parts[0] == "list":
                for name, q in poses.items():
                    print("  %-10s %s" % (name, q))
            elif parts[0] == "snap":
                if camera_loop.latest is not None:
                    fn = "snap_%d.jpg" % int(time.time())
                    cv2.imwrite(fn, camera_loop.latest)
                    print("wrote", fn)
            elif parts[0].startswith("j") and len(parts) == 2:
                i = int(parts[0][1:]) - 1
                q = list(joints)
                q[i] = float(parts[1])
                move(q)
            else:
                vals = [float(v) for v in parts]
                if len(vals) not in (5, 6):
                    print("enter 5 or 6 angles")
                    continue
                move(vals + joints[5:6] if len(vals) == 5 else vals)
        except (ValueError, IndexError) as e:
            print("bad input:", e)

    running = False
    time.sleep(0.2)


if __name__ == "__main__":
    main()
