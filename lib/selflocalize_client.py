"""Live self-localization plotter. Runs on the LAPTOP (has matplotlib + a display).

Start this first, then run selflocalize.py on the Pi with PLOT_HOST set to this
machine's IP. Listens on all interfaces on port 5005 (override with PLOT_PORT).

Protocol: one JSON state per line (newline-delimited), as sent by
lib/netplot.PlotSender. Expected keys (all optional, drawn if present):
    "particles": [[x, y, theta, weight], ...]   # world coords [cm], theta [rad]
    "est_pose":  [x, y, theta]
    "landmarks": {"<id>": [x, y], ...}
"""
import json
import math
import os
import socket

import matplotlib.pyplot as plt

PORT = int(os.environ.get("PLOT_PORT", "5005"))


def main():
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", PORT))
    srv.listen(1)
    print("Waiting for Pi on port {} ...".format(PORT))
    conn, addr = srv.accept()
    print("Connected:", addr)
    buf = conn.makefile("r")

    plt.ion()
    fig, ax = plt.subplots(figsize=(10, 10))
    jet = plt.get_cmap("jet")  # cm.get_cmap was removed in matplotlib 3.9+

    # Fixed world window [cm] so the view stays large and doesn't rescale
    # every frame. Covers the initial particle spread (x in [-100, 500],
    # y in [-250, 350]) plus the landmarks, with a margin.
    XLIM = (-150, 550)
    YLIM = (-300, 400)

    for line in buf:  # one JSON state per line
        line = line.strip()
        if not line:
            continue
        s = json.loads(line)

        ax.clear()

        particles = s.get("particles", [])
        if particles:
            xs = [p[0] for p in particles]
            ys = [p[1] for p in particles]
            us = [15.0 * math.cos(p[2]) for p in particles]
            vs = [15.0 * math.sin(p[2]) for p in particles]
            max_w = max((p[3] for p in particles), default=0.0) or 1.0
            colours = [jet(p[3] / max_w) for p in particles]
            ax.quiver(xs, ys, us, vs, color=colours,
                      angles="xy", scale_units="xy", scale=1, width=0.003)

        for lm_id, (lx, ly) in s.get("landmarks", {}).items():
            ax.add_patch(plt.Circle((lx, ly), 10, fill=False, color="k"))
            ax.annotate(str(lm_id), (lx, ly))

        est = s.get("est_pose")
        if est:
            ax.plot(est[0], est[1], "o", color="magenta", markersize=8)
            ax.quiver([est[0]], [est[1]],
                      [25.0 * math.cos(est[2])],
                      [25.0 * math.sin(est[2])],
                      color="magenta", angles="xy", scale_units="xy", scale=1)

        ax.set_title("Self-localization world view")
        ax.set_aspect("equal")
        ax.set_xlim(*XLIM)
        ax.set_ylim(*YLIM)
        ax.grid(True)
        plt.pause(0.001)

    print("Pi disconnected.")
    plt.ioff()
    plt.show()


if __name__ == "__main__":
    main()
