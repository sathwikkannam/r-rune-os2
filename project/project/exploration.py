import math
import random

import rclpy

from rclpy.node import Node
import numpy as np
from project.srv import GetBestNodeForExploration


class ExplorationService(Node):
    OCCUPIED = 100
    FREE = 2
    UNEXPLORED = -1
    occupancy_grid, height, width, resolution = None, None, None, None

    def __int__(self):
        super().__init__('exploration_service')
        self.srv = self.create_service(GetBestNodeForExploration, 'explore', self.get_best_node_callback)

    def get_best_node_callback(self, request, response):
        self.width = request.map.info.width
        self.height = request.map.info.height
        self.occupancy_grid = np.array(request.map.data).reshape((self.height, self.width))  # Reshape the 1D data
        self.resolution = request.map.info.resolution

        prm_nodes = self.generate_prm()
        gain_nodes = self.compute_information_gain(prm_nodes)

        best_node = gain_nodes[0][0]

        response.best_node = [best_node[0], best_node[1]]
        return response

    def generate_prm(self, num_nodes: int = 10) -> list:
        """
        Create a set of randomly sampled nodes within the known area of the map.
        :param num_nodes: Number of nodes to sample
        :return: PRM nodes
        """
        nodes = set()

        while len(nodes) != num_nodes:
            x = random.randint(0, self.height - 1)
            y = random.randint(0, self.width - 1)

            if self.occupancy_grid[x, y] not in (self.OCCUPIED, self.UNEXPLORED):
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

    def compute_visible_unknown(self, node: tuple, max_distance: int = 3, directions: int = 36):
        """
        For a given node, count how many unknown cells (value -1)
        are visible by casting rays in 36 directions up to max_distance.

        :param node: A node with x and y (meters).
        :param max_distance: Maximum distance (in meters) for each ray.
        :param directions:
        :return: Total number of unknown cells encountered.
        """
        max_cells = int(max_distance / self.resolution)

        total_unknown = 0
        for d in range(directions):
            angle = 2 * math.pi * d / directions
            total_unknown += self.ray_cast_unknown(node, angle, max_cells)
        return total_unknown

    def ray_cast_unknown(self, node: tuple, angle: float, max_cells: int = 3) -> int:
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
            rx = origin_x + step * self.resolution * math.cos(angle)
            ry = origin_y + step * self.resolution * math.sin(angle)
            col = int(rx / self.resolution)
            row = int(ry / self.resolution)
            if not self.is_valid((row, col)):
                break

            val = self.occupancy_grid[row, col]

            if val == self.UNEXPLORED:
                count += 1
        return count

    def is_valid(self, node: tuple) -> bool:
        """
        :param node: A node in the occupancy_grid
        :return: whether the node is within the limits of the map, and the node is free (not an obstacle)
        """
        return node[0] in range(self.height) and node[1] in range(self.width) and self.occupancy_grid[node] not in (self.OCCUPIED, self.UNEXPLORED)


def main():
    rclpy.init()
    exploration_service = ExplorationService()
    rclpy.spin(exploration_service)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
