#!/usr/bin/env python3
import math, time

# --- Parametreler ---
WORLD   = "corridor_2"   # gz topic -l | grep set_pose ile doğrula
MODEL   = "capsule_link"
CENTER  = (0.0, 0.0, 0.10)
A_SEMI  = 2.0
B_SEMI  = 1.0
ANG_VEL = 0.25
FACE_TANGENT = True
HZ = 50.0

from gz.transport13 import Node
from gz.msgs10.pose_pb2 import Pose

TOPIC = f"/world/{WORLD}/set_pose"

def main():
    node = Node()
    # advertise() -> Publisher döner
    pub = node.advertise(TOPIC, Pose)
    if not pub:
        raise RuntimeError(f"Advertise başarısız: {TOPIC}. WORLD adını kontrol et.")

    print(f"[move_capsule_ellipse] publishing to {TOPIC} for '{MODEL}' (Ctrl+C ile durdur)")
    t0 = time.time()
    dt = 1.0 / HZ

    while True:
        t = time.time() - t0
        th = (ANG_VEL * t) % (2.0 * math.pi)

        x = CENTER[0] + A_SEMI * math.cos(th)
        y = CENTER[1] + B_SEMI * math.sin(th)
        z = CENTER[2]

        if FACE_TANGENT:
            dx = -A_SEMI * math.sin(th) * ANG_VEL
            dy =  B_SEMI * math.cos(th) * ANG_VEL
            yaw = math.atan2(dy, dx)
        else:
            yaw = 0.0

        msg = Pose()
        msg.name = MODEL
        msg.position.x, msg.position.y, msg.position.z = x, y, z
        msg.orientation.z = math.sin(yaw/2.0)
        msg.orientation.w = math.cos(yaw/2.0)

        pub.publish(msg)   # publisher üzerinden publish
        time.sleep(dt)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
