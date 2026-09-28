"""Live RRT plotter. Runs on the LAPTOP (has matplotlib + a display).

Start this ONCE and leave it running. It keeps listening across runs: each
time ex4-2.py runs on the Pi it reconnects and the plot updates in place.
Listens on all interfaces on port 5005 (override with PLOT_PORT). Ctrl-C to quit.
"""
import json
import os
import socket

import matplotlib.pyplot as plt

PORT = int(os.environ.get("PLOT_PORT", "5005"))


def draw(ax, s):
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


def main():
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", PORT))
    srv.listen(1)

    plt.ion()
    fig, ax = plt.subplots(figsize=(8, 8))

    print("Listening on port {}. Leave this running; Ctrl-C to quit.".format(PORT))
    try:
        while True:  # keep accepting new runs from the Pi
            print("Waiting for Pi ...")
            conn, addr = srv.accept()
            print("Connected:", addr)
            buf = conn.makefile("r")
            for line in buf:  # one JSON state per line
                line = line.strip()
                if not line:
                    continue
                draw(ax, json.loads(line))
            conn.close()
            print("Pi disconnected; ready for next run.")
    except KeyboardInterrupt:
        print("\nBye.")
    finally:
        srv.close()


if __name__ == "__main__":
    main()
