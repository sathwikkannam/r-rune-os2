import rclpy

from rclpy.node import Node
import numpy as np
import heapq
from my_services.srv import PathPlanner


class NavigationService(Node):
    OCCUPIED = 100
    FREE = 2
    UNEXPLORED = -1

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

    def __init__(self):
        super().__init__('navigation_service')
        self.srv = self.create_service(PathPlanner, '/navigate', self.get_optimal_path_callback)

    def get_optimal_path_callback(self, request, response):
        """
        The request:
            map = OccupancyGrid() -> data, info.height, info.width, header (frame_id, stamp)
            start = tuple
            end = tuple
            path =  list (this is the response)
        Find the shortest path using Dijkstra's algorithm.
        """

        start: tuple = (request.start[0], request.start[1])
        end: tuple = (request.end[0], request.end[1])
        width: int = request.map.info.width
        height: int = request.map.info.height
        occupancy_grid: np.ndarray = np.array(request.map.data).reshape((height, width))  # Reshape the 1D data

        distances = np.full((height, width), np.inf)
        visited = np.zeros((height, width), dtype=bool)

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
                if self.is_valid(neighbor, width, height, occupancy_grid) and not visited[neighbor]:
                    distance = current_distance + 1
                    if distance < distances[neighbor]:
                        distances[neighbor] = distance
                        previous[neighbor] = current
                        heapq.heappush(priority_queue, (distance, neighbor))

        path_tuples = self.backtrack(end, previous)
        response.path = [coord for point in path_tuples for coord in point]  # Flatten the list of tuples
        return response

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

    def is_valid(self, node: tuple, width: int, height: int, occupancy_grid: np.ndarray) -> bool:
        """
        :param occupancy_grid:
        :param height:
        :param width:
        :param node: A node in the occupancy_grid
        :return: whether the node is within the limits of the map, and the node is free (not an obstacle)
        """
        return node[0] in range(height) and node[1] in range(width) and occupancy_grid[node] not in (self.OCCUPIED,
                                                                                                     self.UNEXPLORED)


def main():
    rclpy.init()
    navigation_service = NavigationService()
    rclpy.spin(navigation_service)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
