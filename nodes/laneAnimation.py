#!/usr/bin/env python3
import cv2
import numpy as np

def bezier_point(p0, p1, p2, t):
    return (
        int((1-t)**2 * p0[0] + 2*(1-t)*t*p1[0] + t**2 * p2[0]),
        int((1-t)**2 * p0[1] + 2*(1-t)*t*p1[1] + t**2 * p2[1])
    )

def draw_curved_dash_line(overlay, h, w, base_x, base_y, vp_x, vp_y, sway,
                          dash_len=0.05, spacing=0.15, color=(0,255,0)):
    ctrl_x = base_x + sway
    ctrl_y = (base_y + vp_y) // 2
    p0, p1, p2 = (base_x, base_y), (ctrl_x, ctrl_y), (vp_x, vp_y)

    t = 0
    while t < 1:
        t0, t1 = t, min(1, t + dash_len)
        x0, y0 = bezier_point(p0, p1, p2, t0)
        x1, y1 = bezier_point(p0, p1, p2, t1)
        thickness = max(1, int(10 * (1 - t0)))
        cv2.line(overlay, (x0, y0), (x1, y1), color, thickness)
        t += dash_len + spacing

def main():
    cap = cv2.VideoCapture(2)  # <-- /dev/video5
    if not cap.isOpened():
        print("Error: Could not open camera at /dev/video5")
        return

    # Lucas–Kanade params
    lk_params = dict(winSize=(15, 15), maxLevel=2,
                     criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

    # Feature detection params
    feature_params = dict(maxCorners=100, qualityLevel=0.3, minDistance=7, blockSize=7)

    # First frame
    ret, old_frame = cap.read()
    if not ret:
        print("Error: No frame")
        return
    old_gray = cv2.cvtColor(old_frame, cv2.COLOR_BGR2GRAY)
    p0 = cv2.goodFeaturesToTrack(old_gray, mask=None, **feature_params)

    sway = 0
    sway_smooth = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            break
        frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Track points
        p1, st, err = cv2.calcOpticalFlowPyrLK(old_gray, frame_gray, p0, None, **lk_params)

        if p1 is not None and st.sum() > 0:
            good_new = p1[st == 1]
            good_old = p0[st == 1]

            # Average horizontal displacement
            dx = np.mean(good_new[:, 0] - good_old[:, 0])
            sway = int(dx * 5)  # scale factor

            # Smooth sway
            sway_smooth = int(0.8 * sway_smooth + 0.2 * sway)

        # Update for next iteration
        old_gray = frame_gray.copy()
        p0 = cv2.goodFeaturesToTrack(old_gray, mask=None, **feature_params)

        h, w, _ = frame.shape
        overlay = frame.copy()

        # Vanishing point
        vp_x, vp_y = w // 2, int(h * 0.3)
        base_x, base_y = w // 2, h

        # Draw single dashed center line with sway
        draw_curved_dash_line(overlay, h, w, base_x, base_y, vp_x, vp_y, sway_smooth)

        # Blend overlay
        frame = cv2.addWeighted(overlay, 0.9, frame, 0.1, 0)
        cv2.imshow("Sparse Optical Flow Lane", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

