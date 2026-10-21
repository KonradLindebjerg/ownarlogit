"""RRT path planner in the single MAP frame (cm, radians CCW from +x).

Adapted from the course ex4 planner. The important change from rrt_pi.py is
consistency: this module's geometry AND its `robot_orientation` are both in the
map convention (+x forward, CCW+, radians) - the same one the particle filter
and the drive loop use. There is no second (+y-forward / degrees) convention
hiding in here, which is what made the original rrt_pi mix break.

Obstacles are (x, y, radius, id) in map coordinates. `planning()` returns the
path GOAL-first (goal ... start); the drive loop reverses it.
"""

import math
import random


class RRT:
    class Node:
        def __init__(self, x, y):
            self.x = x
            self.y = y
            self.path_x = []
            self.path_y = []
            self.parent = None

    class AreaBounds:
        def __init__(self, area):
            self.xmin, self.xmax = float(area[0]), float(area[1])
            self.ymin, self.ymax = float(area[2]), float(area[3])

    def __init__(self, start, goal, obstacle_list, rand_area,
                 expand_dis=30.0, path_resolution=5.0, goal_sample_rate=5,
                 max_iter=500, play_area=None, robot_radius=0.0,
                 robot_orientation=0.0, min_turn_angle=5.0, straight_tol=2.0,
                 plot_sender=None):
        self.start = self.Node(start[0], start[1])
        self.end = self.Node(goal[0], goal[1])
        self.min_rand, self.max_rand = rand_area[0], rand_area[1]
        self.play_area = self.AreaBounds(play_area) if play_area else None
        self.expand_dis = expand_dis
        self.path_resolution = path_resolution
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.obstacle_list = obstacle_list
        self.node_list = []
        self.robot_radius = robot_radius
        self.robot_orientation = robot_orientation  # radians, map frame (+x, CCW+)
        # Reject tiny turns the robot cannot execute reliably: a branch is kept
        # only if it continues ~straight (<= straight_tol) or turns for real
        # (>= min_turn_angle). Both in degrees.
        self.min_turn_angle = min_turn_angle
        self.straight_tol = straight_tol
        self.position = [self.start.x, self.start.y]
        self.plot_sender = plot_sender

    def planning(self, animation=True):
        self.node_list = [self.start]
        for i in range(self.max_iter):
            rnd = self.get_random_node()
            nearest = self.node_list[self.get_nearest_node_index(self.node_list, rnd)]
            new_node = self.steer(nearest, rnd, self.expand_dis)

            turn_angle = self.calc_turn_angle(nearest, new_node)
            turn_ok = (turn_angle <= self.straight_tol
                       or turn_angle >= self.min_turn_angle)

            if (self.inside_play_area(new_node)
                    and self.collision_free(new_node)
                    and turn_ok):
                self.node_list.append(new_node)

            if animation and i % 5 == 0:
                self.draw_graph(rnd)

            last = self.node_list[-1]
            if self.calc_dist_to_goal(last.x, last.y) <= self.expand_dis:
                final = self.steer(last, self.end, self.expand_dis)
                if self.collision_free(final):
                    self.node_list.append(final)
                    path = self.generate_final_course(len(self.node_list) - 1)
                    if animation:
                        self.draw_graph(path=path)
                    return path
        return None  # no path found

    def steer(self, from_node, to_node, extend_length=float("inf")):
        new_node = self.Node(from_node.x, from_node.y)
        d, theta = self.calc_distance_and_angle(new_node, to_node)
        new_node.path_x = [new_node.x]
        new_node.path_y = [new_node.y]

        extend_length = min(extend_length, d)
        n_expand = int(math.floor(extend_length / self.path_resolution))
        for _ in range(n_expand):
            new_node.x += self.path_resolution * math.cos(theta)
            new_node.y += self.path_resolution * math.sin(theta)
            new_node.path_x.append(new_node.x)
            new_node.path_y.append(new_node.y)

        d, _ = self.calc_distance_and_angle(new_node, to_node)
        if d <= self.path_resolution:
            new_node.path_x.append(to_node.x)
            new_node.path_y.append(to_node.y)
            new_node.x, new_node.y = to_node.x, to_node.y

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
        return math.hypot(x - self.end.x, y - self.end.y)

    def get_random_node(self):
        if random.randint(0, 100) > self.goal_sample_rate:
            return self.Node(random.uniform(self.min_rand, self.max_rand),
                             random.uniform(self.min_rand, self.max_rand))
        return self.Node(self.end.x, self.end.y)

    def inside_play_area(self, node):
        if self.play_area is None:
            return True
        return (self.play_area.xmin <= node.x <= self.play_area.xmax
                and self.play_area.ymin <= node.y <= self.play_area.ymax)

    def collision_free(self, node):
        if node is None:
            return False
        for (ox, oy, size, _oid) in self.obstacle_list:
            d2 = [(ox - x) ** 2 + (oy - y) ** 2
                  for x, y in zip(node.path_x, node.path_y)]
            if d2 and min(d2) <= (size + self.robot_radius) ** 2:
                return False
        return True

    @staticmethod
    def get_nearest_node_index(node_list, rnd):
        d = [(n.x - rnd.x) ** 2 + (n.y - rnd.y) ** 2 for n in node_list]
        return d.index(min(d))

    @staticmethod
    def calc_distance_and_angle(from_node, to_node):
        dx, dy = to_node.x - from_node.x, to_node.y - from_node.y
        return math.hypot(dx, dy), math.atan2(dy, dx)

    def calc_turn_angle(self, nearest, new_node):
        """Heading change (deg, 0..180) at `nearest` continuing to `new_node`.

        Incoming heading is the edge from `nearest`'s parent, or - at the start
        node - the robot's current orientation. All in map-frame radians.
        """
        _, new_heading = self.calc_distance_and_angle(nearest, new_node)
        if nearest.parent is not None:
            _, in_heading = self.calc_distance_and_angle(nearest.parent, nearest)
        else:
            in_heading = self.robot_orientation
        diff = math.atan2(math.sin(new_heading - in_heading),
                          math.cos(new_heading - in_heading))
        return abs(math.degrees(diff))

    def draw_graph(self, rnd=None, path=None):
        """Stream the current tree to the laptop plot client, if connected."""
        if self.plot_sender is None:
            return
        self.plot_sender.send({
            "edges": [[n.path_x, n.path_y] for n in self.node_list if n.parent],
            "obstacles": [[ox, oy, size, oid]
                          for (ox, oy, size, oid) in self.obstacle_list],
            "start": [self.start.x, self.start.y],
            "end": [self.end.x, self.end.y],
            "rnd": None if rnd is None else [rnd.x, rnd.y],
            "robot_radius": self.robot_radius,
            "path": path,
        })
