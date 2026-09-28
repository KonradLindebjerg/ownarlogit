import os

import rrt_pi as rrt
import landmarkmapping as lm
from netplot import PlotSender

# IP of the laptop running plot_client.py. Override with:  PLOT_HOST=<ip> python ex4-2.py
PLOT_HOST = os.environ.get("PLOT_HOST", "172.20.10.3")
PLOT_PORT = int(os.environ.get("PLOT_PORT", "5005"))


def main(gx=6.0, gy=10.0):
    # Connect to the laptop's live plotter (headless-safe: runs anyway if it fails).
    sender = PlotSender(PLOT_HOST, PLOT_PORT)

    obstacleList = lm.detectLandmark()

    # Set Initial parameters
    rrttest = rrt.RRT(
        start=[0, 0],
        goal=[gx, gy],
        rand_area=[-2, 15],
        obstacle_list=obstacleList,
        # play_area=[0, 10, 0, 14]
        robot_radius=0.225,
        plot_sender=sender,
    )

    path = rrttest.planning(animation=True)

    if path is None:
        print("Cannot find path")
    else:
        print("found path!!")
        # Push the final graph with the solution path to the laptop plot.
        rrttest.draw_graph(path=path)

    sender.close()
    return path


if __name__ == "__main__":
    main()
