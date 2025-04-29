import rclpy

from rclpy.node import Node
from project.srv import PathPlanner
from project.srv import GetBestNodeForExploration
from rclpy.action import ActionClient
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
import inspect


class ExplorerNavigator(Node):

    def __init__(self):
        super().__init__('explorer_navigator')

        self.current_map = None
        self.robot_pose = None
        self.current_grid_pose = None
        self.current_exploration_goal_grid = None
        self.current_navigation_path_grid = None
        self.current_navigation_goal_map = None

        self.state = 'IDLE'

        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.robot_pose_sub = self.create_subscription(OccupancyGrid, '/amcl_pose', self.robot_pose_callback, 10)
        self.navigation_client = self.create_client(PathPlanner, 'navigate')
        self.exploration_client = self.create_client(GetBestNodeForExploration, 'explore')
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        while not self.navigation_client.wait_for_service(
                timeout_sec=1.0) or not self.exploration_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Exploration and Navigation services are not available, waiting....')

        self.timer = self.create_timer(1.0, self.constantly_explore)

    def constantly_explore(self):
        if self.state == 'IDLE':
            # Check if map and pose are available and trigger exploration
            if self.current_map and self.robot_pose:
                self.trigger_exploration()
            else:
                self.state = 'WAITING_FOR_MAP_OR_POSE'

    def robot_pose_callback(self, msg):
        pass

    def map_callback(self, msg):
        self.current_map = msg
        if self.state == 'IDLE' or self.state == 'WAITING_FOR_MAP':
            self.trigger_exploration()

    def trigger_exploration(self):
        if not self.current_map:
            self.state = 'WAITING_FOR_MAP'
            return

        self.state = 'EXPLORING'
        exploration_request = GetBestNodeForExploration().request
        exploration_request.map = self.current_map
        future = self.exploration_client.call_async(exploration_request)
        future.add_done_callback(self.exploration_response_callback)

    def exploration_response_callback(self, future):
        try:
            response = future.result()
            if response.best_node:
                self.current_exploration_goal_grid = (response.best_node[0], response.best_node[1])
                self.trigger_navigation()
            else:
                self.state = 'IDLE'
        except Exception:
            self.get_logger().warn(f'Exception in function {inspect.currentframe().f_code.co_name}')
            self.state = 'IDLE'

    def trigger_navigation(self):
        if not self.current_map or not self.current_grid_pose or not self.current_exploration_goal_grid:
            self.state = 'IDLE'
            return

        self.state = 'NAVIGATING'

        navigation_request = PathPlanner().request
        navigation_request.map = self.current_map
        navigation_request.start = [self.current_grid_pose[0], self.current_grid_pose[1]]
        navigation_request.end = [self.current_exploration_goal_grid[0], self.current_exploration_goal_grid[1]]

        future = self.navigation_client.call_async(navigation_request)
        future.add_done_callback(self.navigation_response_callback)

    def navigation_response_callback(self, future):
        try:
            response = future.result()
            if response.path:
                self.current_navigation_path_grid = [(response.path[i], response.path[i + 1]) for i in
                                                     range(0, len(response.path), 2)]
                self.send_path_to_robot_driver(self.current_navigation_path_grid)
            else:
                self.state = 'IDLE'
                self.trigger_exploration()
        except Exception:
            self.state = 'IDLE'
            self.get_logger().warn(f'Exception in function {inspect.currentframe().f_code.co_name}')
            self.trigger_exploration()

    def send_path_to_robot_driver(self, nav_path):
        if not nav_path or self.current_map:
            self.get_logger().warn(f'Got nav_len: {nav_path} or map: {len(self.current_map)} in function {inspect.currentframe().f_code.co_name}')
            self.state = 'IDLE'
            return

        robot_poses_for_path = []
        for grid_point in nav_path:
            map_x, map_y = self.grid_to_map(grid_point[0], grid_point[1])
            pose = PoseStamped()
            pose.header.frame_id = self.current_map.header.frame_id
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = map_x
            pose.pose.position.y = map_y
            pose.pose.orientation.w = 1.0
            robot_poses_for_path.append(pose)

        self.state = 'NAVIGATING'

        for pose in robot_poses_for_path:
            self.nav_to_pose_client.send_goal_async(pose)

        self.state = 'IDLE'


    def grid_to_map(self, grid_row, grid_col):
        """
        Here we need to scale the 2D matrix of the OccupancyGrid to actual points on the map frame.
        :param grid_row:
        :param grid_col:
        :return:
        """
        info = self.current_map.info

        map_x = info.origin.position.x + (grid_col + 0.5) * info.resolution
        map_y = info.origin.position.y + (grid_row + 0.5) * info.resolution
        return map_x, map_y


def main(args=None):
    rclpy.init(args=args)
    explorer_navigator = ExplorerNavigator()
    rclpy.spin(explorer_navigator)
    explorer_navigator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
