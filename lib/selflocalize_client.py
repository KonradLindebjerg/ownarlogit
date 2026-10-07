"""Live self-localization plotter. Runs on the LAPTOP (has a display).

Start this first, then run selflocalize.py on the Pi with PLOT_HOST set to this
machine's IP. Listens on all interfaces on port 5005 (override with PLOT_PORT).

Protocol: one JSON state per line (newline-delimited), as sent by
lib/netplot.PlotSender. Expected keys (all optional, drawn if present):
    "particles": [[x, y, theta, weight], ...]   # world coords [cm], theta [rad]
    "est_pose":  [x, y, theta]
    "landmarks": {"<id>": [x, y], ...}

Drawing now mirrors draw_world() in ex5/selflocalize.py: it paints into a
500x500 cv2 image using the same world->screen transform (flipped y-axis),
the same jet() weight colouring, and the same particle/landmark/pose marks.
"""
import json
import os
import socket

import cv2
import numpy as np

# import math
# import matplotlib.pyplot as plt

PORT = int(os.environ.get("PLOT_PORT", "5005"))

# Some color constants in BGR format (as used by draw_world in selflocalize.py)
CRED     = (0, 0, 255)
CGREEN   = (0, 255, 0)
CBLUE    = (255, 0, 0)
CCYAN    = (255, 255, 0)
CYELLOW  = (0, 255, 255)
CMAGENTA = (255, 0, 255)
CWHITE   = (255, 255, 255)
CBLACK   = (0, 0, 0)

# Colours used when drawing the landmarks, applied in landmark-id sorted order
# to match draw_world's landmark_colors = [CRED, CGREEN].
landmark_colors = [CRED, CGREEN, CBLUE, CYELLOW, CCYAN]

# Fixed world window [cm] so the whole particle spread stays visible (covers the
# initial spread x in [-100, 500], y in [-250, 350] plus landmarks, with a
# margin). These match the old matplotlib XLIM/YLIM. The canvas is sized so that
# 1 world cm == 1 pixel, and the offsets shift the window's min corner to 0 --
# that way draw_world's native marker sizes (radius 2/5, 15 cm heading) are kept.
XLIM = (-150, 550)
YLIM = (-300, 400)
WORLD_W = XLIM[1] - XLIM[0]      # 700
WORLD_H = YLIM[1] - YLIM[0]      # 700
OFFSET_X = -XLIM[0]              # 150
OFFSET_Y = -YLIM[0]              # 300


def jet(x):
    """Colour map for drawing particles. This function determines the colour of
    a particle from its weight."""
    r = (x >= 3.0/8.0 and x < 5.0/8.0) * (4.0 * x - 3.0/2.0) + (x >= 5.0/8.0 and x < 7.0/8.0) + (x >= 7.0/8.0) * (-4.0 * x + 9.0/2.0)
    g = (x >= 1.0/8.0 and x < 3.0/8.0) * (4.0 * x - 1.0/2.0) + (x >= 3.0/8.0 and x < 5.0/8.0) + (x >= 5.0/8.0 and x < 7.0/8.0) * (-4.0 * x + 7.0/2.0)
    b = (x < 1.0/8.0) * (4.0 * x + 1.0/2.0) + (x >= 1.0/8.0 and x < 3.0/8.0) + (x >= 3.0/8.0 and x < 5.0/8.0) * (-4.0 * x + 5.0/2.0)

    return (255.0*r, 255.0*g, 255.0*b)


def draw_world(est_pose, particles, landmarks, world):
    """Visualization.
    This functions draws the robot's position in the world coordinate system.

    Mirrors draw_world() in ex5/selflocalize.py, but reads the state that was
    sent over the wire (plain lists/dicts) instead of Particle/pose objects:
        est_pose  -- [x, y, theta] or None
        particles -- [[x, y, theta, weight], ...]
        landmarks -- {"<id>": [x, y], ...}
        world     -- the cv2 image to paint into
    """

    # Fix the origin of the coordinate system. Unlike the on-robot draw_world
    # (which uses 0, 0), we shift by the window's min corner so negative world
    # coordinates are still drawn inside the canvas.
    offsetX = OFFSET_X
    offsetY = OFFSET_Y

    # Constant needed for transforming from world coordinates to screen coordinates (flip the y-axis)
    ymax = world.shape[0]

    world[:] = CWHITE  # Clear background to white

    # Find largest weight
    max_weight = 0
    for p in particles:
        max_weight = max(max_weight, p[3])
    if max_weight <= 0:
        max_weight = 1.0

    # Draw particles
    for p in particles:
        px, py, ptheta, pweight = p[0], p[1], p[2], p[3]
        x = int(px + offsetX)
        y = ymax - (int(py + offsetY))
        colour = jet(pweight / max_weight)
        cv2.circle(world, (x, y), 2, colour, 2)
        b = (int(px + 15.0*np.cos(ptheta)) + offsetX,
             ymax - (int(py + 15.0*np.sin(ptheta)) + offsetY))
        cv2.line(world, (x, y), b, colour, 2)

    # Draw landmarks
    for i, lm_id in enumerate(sorted(landmarks.keys())):
        lx, ly = landmarks[lm_id]
        lm = (int(lx + offsetX), int(ymax - (ly + offsetY)))
        colour = landmark_colors[i % len(landmark_colors)]
        cv2.circle(world, lm, 5, colour, 2)

    # Draw estimated robot pose
    if est_pose:
        ex, ey, etheta = est_pose[0], est_pose[1], est_pose[2]
        a = (int(ex) + offsetX, ymax - (int(ey) + offsetY))
        b = (int(ex + 15.0*np.cos(etheta)) + offsetX,
             ymax - (int(ey + 15.0*np.sin(etheta)) + offsetY))
        cv2.circle(world, a, 5, CMAGENTA, 2)
        cv2.line(world, a, b, CMAGENTA, 2)


def main():
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", PORT))
    srv.listen(1)
    print("Waiting for Pi on port {} ...".format(PORT))
    conn, addr = srv.accept()
    print("Connected:", addr)
    buf = conn.makefile("r")

    WIN_World = "Self-localization world view"
    cv2.namedWindow(WIN_World)

    # Allocate space for world map. Larger than the on-robot 500x500 so the
    # full window (XLIM x YLIM) fits with 1 cm == 1 pixel.
    world = np.zeros((WORLD_H, WORLD_W, 3), dtype=np.uint8)

    for line in buf:  # one JSON state per line
        line = line.strip()
        if not line:
            continue
        s = json.loads(line)

        particles = s.get("particles", [])
        landmarks = s.get("landmarks", {})
        est = s.get("est_pose")

        draw_world(est, particles, landmarks, world)

        cv2.imshow(WIN_World, world)
        # waitKey is required for the window to actually refresh; also lets the
        # user quit with 'q'.
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    print("Pi disconnected.")
    cv2.destroyAllWindows()


# --- Old matplotlib-based plotter (kept for reference) -----------------------
# def main():
#     srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#     srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
#     srv.bind(("0.0.0.0", PORT))
#     srv.listen(1)
#     print("Waiting for Pi on port {} ...".format(PORT))
#     conn, addr = srv.accept()
#     print("Connected:", addr)
#     buf = conn.makefile("r")
#
#     plt.ion()
#     fig, ax = plt.subplots(figsize=(10, 10))
#     jet = plt.get_cmap("jet")  # cm.get_cmap was removed in matplotlib 3.9+
#
#     # Fixed world window [cm] so the view stays large and doesn't rescale
#     # every frame. Covers the initial particle spread (x in [-100, 500],
#     # y in [-250, 350]) plus the landmarks, with a margin.
#     XLIM = (-150, 550)
#     YLIM = (-300, 400)
#
#     for line in buf:  # one JSON state per line
#         line = line.strip()
#         if not line:
#             continue
#         s = json.loads(line)
#
#         ax.clear()
#
#         particles = s.get("particles", [])
#         if particles:
#             xs = [p[0] for p in particles]
#             ys = [p[1] for p in particles]
#             us = [15.0 * math.cos(p[2]) for p in particles]
#             vs = [15.0 * math.sin(p[2]) for p in particles]
#             max_w = max((p[3] for p in particles), default=0.0) or 1.0
#             colours = [jet(p[3] / max_w) for p in particles]
#             ax.quiver(xs, ys, us, vs, color=colours,
#                       angles="xy", scale_units="xy", scale=1, width=0.003)
#
#         for lm_id, (lx, ly) in s.get("landmarks", {}).items():
#             ax.add_patch(plt.Circle((lx, ly), 10, fill=False, color="k"))
#             ax.annotate(str(lm_id), (lx, ly))
#
#         est = s.get("est_pose")
#         if est:
#             ax.plot(est[0], est[1], "o", color="magenta", markersize=8)
#             ax.quiver([est[0]], [est[1]],
#                       [25.0 * math.cos(est[2])],
#                       [25.0 * math.sin(est[2])],
#                       color="magenta", angles="xy", scale_units="xy", scale=1)
#
#         ax.set_title("Self-localization world view")
#         ax.set_aspect("equal")
#         ax.set_xlim(*XLIM)
#         ax.set_ylim(*YLIM)
#         ax.grid(True)
#         plt.pause(0.001)
#
#     print("Pi disconnected.")
#     plt.ioff()
#     plt.show()


if __name__ == "__main__":
    main()
