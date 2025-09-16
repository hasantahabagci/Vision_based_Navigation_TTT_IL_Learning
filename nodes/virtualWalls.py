#!/usr/bin/env python3
import cv2
import numpy as np

def draw_lane_line(overlay, w, h, vp_x, vp_y, lane_offset, num_dashes=6, spacing=200, dash_height=60, color=(0,255,0)):
    """Draws a perspective-aligned dashed lane line toward the vanishing point."""
    
    # Lane base position at bottom
    base_x = w // 2 + lane_offset
    base_y = h

    # Interpolate along the line toward vanishing point
    for i in range(num_dashes):
        # Bottom and top position of dash along the line
        t0 = i * spacing / (h - vp_y)
        t1 = (i * spacing + dash_height) / (h - vp_y)

        if t0 >= 1:  # past vanishing point
            break

        # Project along the line
        x0 = int((1 - t0) * base_x + t0 * vp_x)
        y0 = int((1 - t0) * base_y + t0 * vp_y)

        x1 = int((1 - t1) * base_x + t1 * vp_x)
        y1 = int((1 - t1) * base_y + t1 * vp_y)

        # Line thickness shrinks with depth
        thickness = max(1, int(10 * (1 - t0)))
        cv2.line(overlay, (x0, y0), (x1, y1), color, thickness)

def main():
    cap = cv2.VideoCapture(5)  # <-- use /dev/video5
    if not cap.isOpened():
        print("Error: Could not open camera at /dev/video5")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        h, w, _ = frame.shape
        overlay = frame.copy()

        # Vanishing point
        vp_x, vp_y = w // 2, int(h * 0.3)

        # Lane offsets (pixels at bottom of image)
        lane_offsets = [-250, 0, 250]

        for offset in lane_offsets:
            draw_lane_line(overlay, w, h, vp_x, vp_y, offset)

        # Blend overlay
        alpha = 0.9
        frame = cv2.addWeighted(overlay, alpha, frame, 1-alpha, 0)

        cv2.imshow("Correct Perspective Lane Markers", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
