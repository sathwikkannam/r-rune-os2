import heapq
import math
import random
import numpy as np
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

class Project(Node):


    def __init__(self):
        super().__init__('navigation_node')
        occupancy_grid: np.ndarray = np.ndarray((100, 100), )
    
        OCCUPIED = 1
        FREE = 2
        UNEXPLORED = 3
        HEIGHT, WIDTH = occupancy_grid.shape
        RESOLUTION = 0.1
        """
            Directions:
            (0, 1): Right
            (1, 0): Down
            (0, -1): Left
            (-1, 0): Up
            (1, 1): Down-Right
            (1, -1): Down-Left
            (-1, 1): Up-Right
            (-1, -1): Up-Left
        """
        directions = [(0, 1), (1, 0), (0, -1), (-1, 0),
                      (1, 1), (1, -1), (-1, 1), (-1, -1)]
    
        robot_pose = ()
    

    def some_callback(self):
        points = self.generate_prm()
        gains = self.compute_information_gain(points)
        node = self.get_best_node(gains)
        self.dijkstra_path(self.robot_pose, node)

    def dijkstra_path(self, start: tuple, end: tuple) -> list:
        """
        Find the shortest path using Dijkstra's algorithm.

        :param start: A tuple representing the starting point (row, col).
        :param end: A tuple representing the ending point (row, col).
        :return: A list of tuples representing the shortest path from the start to the end.
        """
        distances = np.full((self.HEIGHT, self.WIDTH), np.inf)
        visited = np.zeros((self.HEIGHT, self.WIDTH), dtype=bool)

        distances[start] = 0
        priority_queue = [(0, start)]
        previous = {start: None}

        while priority_queue:
            current_distance, current = heapq.heappop(priority_queue)

            if visited[current]:
                continue

            visited[current] = True

            if current == end:
                break

            for direction in self.directions:
                neighbor = (current[0] + direction[0], current[1] + direction[1])
                if self.is_valid(neighbor) and not visited[neighbor]:
                    distance = current_distance + 1
                    if distance < distances[neighbor]:
                        distances[neighbor] = distance
                        previous[neighbor] = current
                        heapq.heappush(priority_queue, (distance, neighbor))

        return self.backtrack(end, previous)

    @staticmethod
    def backtrack(end: tuple, previous: dict) -> list:
        """

        :param end: Goal node
        :param previous:
        :return: A list of nodes starting from goal to the selected node.
        """
        path = []
        current = end
        while current:
            path.append(current)
            current = previous.get(current)
        path.reverse()
        return path

    def is_valid(self, node: tuple) -> bool:
        """
        :param node: A node in the occupancy_grid
        :return: whether the node is within the limits of the map, and the node is free (not an obstacle)
        """
        return node[0] in range(self.HEIGHT) and node[1] in range(self.WIDTH) and self.occupancy_grid[node] == self.FREE

    def generate_prm(self, num_nodes: int = 10) -> list:
        """
        Create a set of randomly sampled nodes within the known area of the map.
        :param num_nodes: Number of nodes to sample
        :return: PRM nodes
        """
        nodes = set()

        while len(nodes) != num_nodes:
            x = random.randint(0, self.HEIGHT - 1)
            y = random.randint(0, self.WIDTH - 1)

            if self.occupancy_grid[x, y] == self.FREE:
                nodes.add((x, y))

        return list(nodes)

    @staticmethod
    def euclidean_distance(node1: tuple, node2: tuple):
        """
        Calculates the Euclidean distance between two nodes
        :param node1: A node in occupancy grid
        :param node2: A node in occupancy grid
        :return: Distance between the two nodes
        """
        return math.sqrt((node2[0] - node1[0]) ** 2 + (node2[1] - node1[1]) ** 2)

    def compute_information_gain(self, prm_nodes: list[tuple], lambda_gain: int = 1) -> dict[tuple:float]:
        """
        Computes the information gain for each PRM nodes
        :param prm_nodes: A set of randomly sampled nodes within the known area of the map.
        :param lambda_gain: Lambda in Information Gain formula.
        :return:
        """
        node_gains = {}
        # tuple: (node, gain)
        previous = ()

        for i, node in enumerate(prm_nodes):
            visible_cells = self.compute_visible_unknown(node)
            if i > 0:
                distance = self.euclidean_distance(node, previous[0])
                gain = previous[1] + visible_cells * math.exp(-lambda_gain * distance)
            else:
                gain = visible_cells  # first node
            node_gains[node] = gain
            previous = (node, gain)

        ranked_nodes = sorted(node_gains.items(), key=lambda item: item[1], reverse=True)
        return ranked_nodes

    @staticmethod
    def get_best_node(ranked_nodes: dict[tuple:float]) -> tuple:
        """
        :param ranked_nodes: A dictionary of nodes ranked based on their gain
        :return: The node with the highest gain
        """
        return ranked_nodes[0]

    def compute_visible_unknown(self, node: tuple, max_distance: int = 3, directions: int = 36):
        """
        For a given node, count how many unknown cells (value -1)
        are visible by casting rays in 36 directions up to max_distance.

        :param node: A node with x and y (meters).
        :param max_distance: Maximum distance (in meters) for each ray.
        :param directions:
        :return: Total number of unknown cells encountered.
        """
        max_cells = int(max_distance / self.RESOLUTION)

        total_unknown = 0
        for d in range(directions):
            angle = 2 * math.pi * d / directions
            total_unknown += self._ray_cast_unknown(node, angle, max_cells)
        return total_unknown

    def _ray_cast_unknown(self, node: tuple, angle: float, max_cells: int = 3) -> int:
        """
        Cast a ray from the node and count unknown cells (-1) until an obstacle (100)
        is encountered or the ray goes out of bounds.
        :param node: A node with x and y (meters).
        :param angle: Angle in radians.
        :param max_cells: Maximum steps along the ray.
        :return: Count of unknown cells seen along this ray.
        """
        origin_x, origin_y = node
        count = 0

        for step in range(1, max_cells + 1):
            rx = origin_x + step * self.RESOLUTION * math.cos(angle)
            ry = origin_y + step * self.RESOLUTION * math.sin(angle)
            col = int(rx / self.RESOLUTION)
            row = int(ry / self.RESOLUTION)
            if not self.is_valid((row, col)):
                break

            val = self.occupancy_grid[row, col]

            if val == self.UNEXPLORED:
                count += 1
        return count
