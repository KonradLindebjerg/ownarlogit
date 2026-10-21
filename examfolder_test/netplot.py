import json
import socket


class PlotSender:
    """Streams RRT plot state over TCP to a matplotlib client on the laptop.

    Run plot_client.py on the laptop first, then run this program on the Pi
    with host set to the laptop's IP. If the connection fails the sender is
    disabled so the planner still runs headless on the Pi."""

    def __init__(self, host, port=5005):
        self.sock = None
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((host, port))
            print("PlotSender connected to {}:{}".format(host, port))
        except OSError as e:
            print("PlotSender: could not connect to {}:{} ({}). "
                  "Running without live plot.".format(host, port, e))
            self.sock = None

    def send(self, state):
        """Send one plot state dict as a newline-delimited JSON message."""
        if self.sock is None:
            return
        try:
            self.sock.sendall((json.dumps(state) + "\n").encode())
        except OSError as e:
            print("PlotSender: send failed ({}); disabling.".format(e))
            self.close()

    def close(self):
        if self.sock is not None:
            try:
                self.sock.close()
            finally:
                self.sock = None
