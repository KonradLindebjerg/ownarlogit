"""Live RRT plotter. Runs on the LAPTOP (has matplotlib + a display).

Start this first, then run ex4-2.py on the Pi with PLOT_HOST set to this
machine's IP. Listens on all interfaces on port 5005 (override with PLOT_PORT).
"""
import json
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
    fig, ax = plt.subplots(figsize=(8, 8))

    for line in buf:  # one JSON state per line
        line = line.strip()
        if not line:
            continue
        s = json.loads(line)

        ax.clear()
        for px, py in s.get("edges", []):
            ax.plot(px, py, "-g")
        for ox, oy, size in s.get("obstacles", []):
            ax.add_patch(plt.Circle((ox, oy), size, fill=False, color="b"))
        if s.get("start"):
            ax.plot(s["start"][0], s["start"][1], "xr")
        if s.get("end"):
            ax.plot(s["end"][0], s["end"][1], "xr")
        if s.get("rnd"):
            ax.plot(s["rnd"][0], s["rnd"][1], "^k")
        if s.get("path"):
            ax.plot([x for (x, y) in s["path"]],
                    [y for (x, y) in s["path"]], "-r", linewidth=2)
        ax.axis("equal")
        ax.grid(True)
        plt.pause(0.001)

    print("Pi disconnected.")
    plt.ioff()
    plt.show()


if __name__ == "__main__":
    main()
