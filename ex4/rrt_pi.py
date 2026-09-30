import math
import random

# NOTE: This is a matplotlib-free variant of rrt.py for running on the Pi,
# which cannot import matplotlib. All local plotting has been removed. To
# visualise the search, pass a `plot_sender` and view it on the laptop client.


class RRT:
    """
    Class for RRT planning
    """

    class Node:
        """
        RRT Node
        """

        def __init__(self, x, y):
            self.x = x
            self.y = y
            self.path_x = []
            self.path_y = []
            self.parent = None

    class AreaBounds:

        def __init__(self, area):
            self.xmin = float(area[0])
            self.xmax = float(area[1])
            self.ymin = float(area[2])
            self.ymax = float(area[3])

    def __init__(self,
                 start,
                 goal,
                 obstacle_list,
                 rand_area,
                 expand_dis=3.0,
                 path_resolution=0.5,
                 goal_sample_rate=5,
                 max_iter=500,
                 play_area=None,
                 robot_radius=0.0,
                 robot_orientation=0.0,
                 min_turn_angle=10.0,
                 straight_tol=2.0,
                 plot_sender=None,
                 ):
        """
        Setting Parameter

        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:Random Sampling Area [min,max]
        play_area:stay inside this area [xmin,xmax,ymin,ymax]
        robot_radius: robot body modeled as circle with given radius

        """
        self.start = self.Node(start[0], start[1])
        self.end = self.Node(goal[0], goal[1])
        self.min_rand = rand_area[0]
        self.max_rand = rand_area[1]
        if play_area is not None:
            self.play_area = self.AreaBounds(play_area)
        else:
            self.play_area = None
        self.expand_dis = expand_dis
        self.path_resolution = path_resolution
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.obstacle_list = obstacle_list
        self.node_list = []
        self.robot_radius = robot_radius
        self.robot_orientation = robot_orientation
        # Turn filtering (degrees): a new edge is only accepted if the turn the
        # robot makes at the branching node is either ~straight (<= straight_tol)
        # or a "real" turn (>= min_turn_angle). Tiny in-between turns are
        # rejected because the robot can't execute them reliably.
        self.min_turn_angle = min_turn_angle
        self.straight_tol = straight_tol
        self.position = [self.start.x, self.start.y]
        self.plot_sender = plot_sender

    def planning(self, animation=True):
        """
        rrt path planning

        animation: flag for animation on or off
        """

        self.node_list = [self.start]
        for i in range(self.max_iter):
            rnd_node = self.get_random_node()
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd_node)
            nearest_node = self.node_list[nearest_ind]

            new_node = self.steer(nearest_node, rnd_node, self.expand_dis)
            turn_angle = self.calc_turn_angle(nearest_node, new_node)
            turn_ok = (turn_angle <= self.straight_tol or
                       turn_angle >= self.min_turn_angle)

            if self.check_if_outside_play_area(new_node, self.play_area) and \
               self.check_collision(
                   new_node, self.obstacle_list, self.robot_radius) and turn_ok:
                self.node_list.append(new_node)

            if animation and i % 5 == 0:
                self.draw_graph(rnd_node)

            if self.calc_dist_to_goal(self.node_list[-1].x,
                                      self.node_list[-1].y) <= self.expand_dis:
                final_node = self.steer(self.node_list[-1], self.end,
                                        self.expand_dis)
                if self.check_collision(
                        final_node, self.obstacle_list, self.robot_radius):
                    return self.generate_final_course(len(self.node_list) - 1)

            if animation and i % 5:
                self.draw_graph(rnd_node)

        return None  # cannot find path

    def steer(self, from_node, to_node, extend_length=float("inf")):

        new_node = self.Node(from_node.x, from_node.y)
        d, theta = self.calc_distance_and_angle(new_node, to_node)

        new_node.path_x = [new_node.x]
        new_node.path_y = [new_node.y]

        if extend_length > d:
            extend_length = d

        n_expand = math.floor(extend_length / self.path_resolution)

        for _ in range(n_expand):
            new_node.x += self.path_resolution * math.cos(theta)
            new_node.y += self.path_resolution * math.sin(theta)
            new_node.path_x.append(new_node.x)
            new_node.path_y.append(new_node.y)

        d, _ = self.calc_distance_and_angle(new_node, to_node)
        if d <= self.path_resolution:
            new_node.path_x.append(to_node.x)
            new_node.path_y.append(to_node.y)
            new_node.x = to_node.x
            new_node.y = to_node.y

        new_node.parent = from_node

        return new_node

    def generate_final_course(self, goal_ind):
        path = [[self.end.x, self.end.y]]
        node = self.node_list[goal_ind]
        while node.parent is not None:
            path.append([node.x, node.y])
            node = node.parent
        path.append([node.x, node.y])

        return path

    def calc_dist_to_goal(self, x, y):
        dx = x - self.end.x
        dy = y - self.end.y
        return math.hypot(dx, dy)

    def get_random_node(self):
        if random.randint(0, 100) > self.goal_sample_rate:
            rnd = self.Node(
                random.uniform(self.min_rand, self.max_rand),
                random.uniform(self.min_rand, self.max_rand))
        else:  # goal point sampling
            rnd = self.Node(self.end.x, self.end.y)
        return rnd

    def draw_graph(self, rnd=None, path=None):
        # Headless (Pi) variant: no local matplotlib rendering. If a network
        # plot sender is configured, stream the state to the laptop client.
        # Otherwise this is a no-op.
        if self.plot_sender is None:
            return

        state = {
            "edges": [[node.path_x, node.path_y]
                      for node in self.node_list if node.parent],
            "obstacles": [[ox, oy, size]
                          for (ox, oy, size) in self.obstacle_list],
            "start": [self.start.x, self.start.y],
            "end": [self.end.x, self.end.y],
            "rnd": None if rnd is None else [rnd.x, rnd.y],
            "robot_radius": self.robot_radius,
            "path": path,
        }
        self.plot_sender.send(state)

    @staticmethod
    def get_nearest_node_index(node_list, rnd_node):
        dlist = [(node.x - rnd_node.x)**2 + (node.y - rnd_node.y)**2
                 for node in node_list]
        minind = dlist.index(min(dlist))

        return minind

    @staticmethod
    def check_if_outside_play_area(node, play_area):

        if play_area is None:
            return True  # no play_area was defined, every pos should be ok

        if node.x < play_area.xmin or node.x > play_area.xmax or \
           node.y < play_area.ymin or node.y > play_area.ymax:
            return False  # outside - bad
        else:
            return True  # inside - ok

    @staticmethod
    def check_collision(node, obstacleList, robot_radius):

        if node is None:
            return False

        for (ox, oy, size) in obstacleList:
            dx_list = [ox - x for x in node.path_x]
            dy_list = [oy - y for y in node.path_y]
            d_list = [dx * dx + dy * dy for (dx, dy) in zip(dx_list, dy_list)]

            if min(d_list) <= (size+robot_radius)**2:
                return False  # collision

        return True  # safe

    @staticmethod
    def calc_distance_and_angle(from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        d = math.hypot(dx, dy)
        theta = math.atan2(dy, dx)
        return d, theta

    def calc_turn_angle(self, nearest_node, new_node):
        """Heading change (degrees, in [0, 180]) the robot makes at
        ``nearest_node`` when it continues on to ``new_node``.

        Compares the heading of the edge entering ``nearest_node`` (from its
        parent) with the heading of the new edge ``nearest_node -> new_node``.
        For the start node, which has no parent, the robot's initial
        orientation is used as the incoming heading.
        """
        _, new_heading = self.calc_distance_and_angle(nearest_node, new_node)

        if nearest_node.parent is not None:
            _, in_heading = self.calc_distance_and_angle(
                nearest_node.parent, nearest_node)
        else:
            in_heading = self.robot_orientation

        # Normalise the difference to (-pi, pi] so wrap-around is handled, then
        # return its magnitude in degrees.
        diff = math.atan2(math.sin(new_heading - in_heading),
                          math.cos(new_heading - in_heading))
        return abs(math.degrees(diff))


def main(gx=6.0, gy=10.0):
    print("start " + __file__)

    # ====Search Path with RRT====
    obstacleList = [(5, 5, 1), (3, 6, 2), (3, 8, 2), (3, 10, 2), (7, 5, 2),
                    (9, 5, 2), (8, 10, 1)]  # [x, y, radius]
    # Set Initial parameters
    rrt = RRT(
        start=[0, 0],
        goal=[gx, gy],
        rand_area=[-2, 15],
        obstacle_list=obstacleList,
        # play_area=[0, 10, 0, 14]
        robot_radius=0.8
        )
    path = rrt.planning(animation=False)

    if path is None:
        print("Cannot find path")
    else:
        print("found path!!")
        print(path)


if __name__ == '__main__':
    main()
