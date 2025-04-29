import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
#from rclpy.action import GoalResponse, GoalStatus as ActionGoalStatus
from action_msgs.msg import GoalStatus as ActionGoalStatus
#from actionlib_msgs.msg import GoalStatus as ActionGoalStatus
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Pose
from nav2_msgs.action import NavigateToPose
from my_services.srv import PathPlanner, GetBestNodeForExploration
from nav_msgs.msg import Odometry

class ExplorerNavigator(Node):

    def __init__(self):
        super().__init__('explorer_navigator')

        self._get_result_future = None
        self.current_map = None
        self.current_map_origin = None
        self.current_map_resolution = None
        self.current_map_height = 0
        self.current_map_width = 0
        self.Z = 0

        self.robot_pose = None  # Pose Data
        self.current_grid_pose = None  # Tuple data

        self.current_exploration_goal_grid = None  # Goal in the grid (tuple)
        self.current_navigation_path_grid = None  # This is the path from navigation.
        self.current_path_index = -1

        self.state = 'IDLE'

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        self.robot_pose_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.robot_pose_callback,
            10
        )

        self.navigation_client = self.create_client(
            PathPlanner,
            '/navigate'
        )

        self.exploration_client = self.create_client(
            GetBestNodeForExploration,
            '/explore'
        )

        self.nav_to_pose_client = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose'
        )

        while not self.navigation_client.wait_for_service(
                timeout_sec=1.0):
            self.get_logger().info('Navigation service are not available, waiting....')

        while not self.exploration_client.wait_for_service(
                timeout_sec=1.0):
            self.get_logger().info('Exploration service are not available, waiting....')

        self.get_logger().info('Exploration and Navigation services are connected')

        self.timer = self.create_timer(1.0, self.constantly_explore)
        self.timer = self.create_timer(5.0, self.show_state)

    def show_state(self):
        # I don't want to colcon build everytime! So show me whats wrong.
        log_msg = f"Current State: {self.state}"
        if self.state == 'FOLLOWING_PATH' and self.current_navigation_path_grid:
            log_msg += f" (Waypoint {self.current_path_index + 1}/{len(self.current_navigation_path_grid)})"
        elif self.state == 'NAVIGATING':
            log_msg += f" (Planning path to {self.current_exploration_goal_grid})"
        elif self.state == 'EXPLORING':
            log_msg += f" (Finding next exploration goal)"

        self.get_logger().info(log_msg)

    def constantly_explore(self):
        if self.state == 'IDLE':
            if self.current_map and self.current_grid_pose:
                self.get_logger().info("Triggering exploration.")
                self.trigger_exploration()
            else:
                self.state = 'WAITING_FOR_MAP_OR_POSE'

    def robot_pose_callback(self, msg):
        self.robot_pose = msg.pose.pose # This is Pose Data.

        #if self.current_map_resolution and self.current_map_origin:
        new_grid_pose = self.map_to_grid(
            self.robot_pose.position.x,
            self.robot_pose.position.y
            # Z also exists, but we don't
        )

        self.Z = self.robot_pose.position.z

        if new_grid_pose != self.current_grid_pose:
            self.current_grid_pose = new_grid_pose

            if self.state == 'WAITING_FOR_MAP_OR_POSE':
                self.state = 'IDLE'

    def map_callback(self, msg):
        # No condition, we just get a map whenever it wants.
        self.current_map = msg
        self.current_map_origin = msg.info.origin
        self.current_map_resolution = msg.info.resolution
        self.current_map_height = msg.info.height
        self.current_map_width = msg.info.width

        if self.state == 'WAITING_FOR_MAP_OR_POSE':
            self.state = 'IDLE'

    def trigger_exploration(self):
        if not self.current_map:
            self.get_logger().warn("no map in trigger_exploration.")
            self.state = 'WAITING_FOR_MAP_OR_POSE'
            return

        self.get_logger().info("Requesting explore.srv")
        self.state = 'EXPLORING'
        exploration_request = GetBestNodeForExploration.Request()
        exploration_request.map = self.current_map

        future = self.exploration_client.call_async(exploration_request)
        future.add_done_callback(self.exploration_response_callback)

    def exploration_response_callback(self, future):
        try:
            response = future.result()
            if response.best_node and len(response.best_node) == 2:
                self.current_exploration_goal_grid = (response.best_node[0], response.best_node[1])
                self.get_logger().info(f"Explore.srv node: {self.current_exploration_goal_grid}")
                self.trigger_navigation()
            else:
                self.get_logger().info(
                    "Invalid node in exploration_response_callback")
                self.state = 'IDLE'
        except Exception as e:
            self.get_logger().error(f'Exception in exploration_response_callback')
            self.state = 'IDLE'

    def trigger_navigation(self):
        if not self.current_grid_pose or not self.current_exploration_goal_grid:
            self.get_logger().warn(f"Something missing in trigger_navigation Pose: {self.current_grid_pose}, Explore node: {self.current_exploration_goal_grid}")
            self.state = 'IDLE'
            return

        self.get_logger().info(f"Navigate.srv from {self.current_grid_pose} to {self.current_exploration_goal_grid}")
        self.state = 'NAVIGATING'

        navigation_request = PathPlanner.Request()
        navigation_request.map = self.current_map
        navigation_request.start = [self.current_grid_pose[0], self.current_grid_pose[1]]
        navigation_request.end = [self.current_exploration_goal_grid[0], self.current_exploration_goal_grid[1]]

        future = self.navigation_client.call_async(navigation_request)
        future.add_done_callback(self.navigation_response_callback)

    def navigation_response_callback(self, future):
        try:
            response = future.result()
            if response.path and len(response.path) > 0:
                self.get_logger().info(f"Navigate.srv path: {len(response.path) // 2} points.")
                # Navigate.srv takes int32[] is 1D array. Convert to list of tuples.
                self.current_navigation_path_grid = [(response.path[i], response.path[i + 1]) for i in
                                                     range(0, len(response.path), 2)]

                if len(self.current_navigation_path_grid) > 1:
                    self.current_path_index = 1
                    self.state = 'FOLLOWING_PATH'
                    self.send_waypoint_goal()
                else:
                    self.state = 'IDLE'
                    self.trigger_exploration()

            else:
                self.get_logger().warn("Got no path in navigation_response_callback")
                self.state = 'IDLE'
                self.trigger_exploration()
        except Exception as e:
            self.get_logger().error(f'Exception in navigation_response_callback: {e}')
            self.state = 'IDLE'
            # self.trigger_exploration()

    def send_waypoint_goal(self):
        if not self.current_navigation_path_grid or \
                not (0 <= self.current_path_index < len(self.current_navigation_path_grid)):
            self.get_logger().error(f"Invalid path or index ({self.current_path_index}) for sending waypoint goal.")
            self.state = 'IDLE'
            self.trigger_exploration()
            return

        if not self.current_map:
            self.get_logger().warn(f'Map not available for sending waypoint goal.')
            self.state = 'IDLE'
            self.trigger_exploration()
            return

        target_grid_point = self.current_navigation_path_grid[self.current_path_index]

        try:
            map_x, map_y = self.grid_to_map(target_grid_point[0], target_grid_point[1])
        except TypeError:
            self.get_logger().error(f"Failed to convert grid point {target_grid_point} to map coordinates.")
            self.state = 'IDLE'
            self.trigger_exploration()
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = self.current_map.header.frame_id
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = map_x
        goal_msg.pose.pose.position.y = map_y
        goal_msg.pose.pose.position.z = self.Z

        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info(
            f"Sending waypoint {self.current_path_index + 1}/{len(self.current_navigation_path_grid)} goal: grid{target_grid_point} -> map({map_x:.2f}, {map_y:.2f})")
        self.state = 'FOLLOWING_PATH'

        if not self.nav_to_pose_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('/navigate_to_pose action server not available')
            self.state = 'IDLE'
            self.trigger_exploration()
            return

        send_goal_future = self.nav_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=self.nav_feedback_callback)

        send_goal_future.add_done_callback(self.nav_goal_response_callback)

    def nav_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().warn('Waypoint navigation goal rejected by Nav2 action server.')
            self.state = 'IDLE'
            self.current_navigation_path_grid = None
            self.current_path_index = -1
            self.trigger_exploration()
            return

        self.get_logger().info('Waypoint navigation goal accepted by Nav2.')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.nav_get_result_callback)

    def nav_get_result_callback(self, future):
        result_wrapper = future.result()
        status = result_wrapper.status

        if status == ActionGoalStatus.STATUS_SUCCEEDED:
            self.current_path_index += 1

            if self.current_path_index < len(self.current_navigation_path_grid):
                self.send_waypoint_goal()
            else:
                self.get_logger().info("Navigation successful :)")
                self.state = 'IDLE'
                self.current_navigation_path_grid = None
                self.current_path_index = -1
                self.trigger_exploration()

        else:
            self.get_logger().warn(
                f'Failed in nav_get_result_callback status: {status}')
            self.state = 'IDLE'
            self.current_navigation_path_grid = None
            self.current_path_index = -1
            self.trigger_exploration()

    def nav_feedback_callback(self, feedback_msg):
        # Does nothing.
        feedback = feedback_msg.feedback

    def grid_to_map(self, grid_row, grid_col):
        if self.current_map_resolution is None or self.current_map_origin is None:
            self.get_logger().error("No map in grid_to_map")
            return None, None

        map_x = self.current_map_origin.position.x + (grid_col + 0.5) * self.current_map_resolution
        map_y = self.current_map_origin.position.y + (grid_row + 0.5) * self.current_map_resolution
        return map_x, map_y

    def map_to_grid(self, map_x, map_y):
        if self.current_map_resolution is None or self.current_map_origin is None or self.current_map_resolution == 0:
            return None

        grid_col_float = (map_x - self.current_map_origin.position.x) / self.current_map_resolution
        grid_row_float = (map_y - self.current_map_origin.position.y) / self.current_map_resolution

        grid_col = int(grid_col_float)
        grid_row = int(grid_row_float)

        if not (0 <= grid_row < self.current_map_height and 0 <= grid_col < self.current_map_width):
            self.get_logger().warn(f"map_to_grid is out of bounds")
            return None

        return grid_row, grid_col


def main(args=None):
    rclpy.init(args=args)
    explorer_navigator = ExplorerNavigator()
    rclpy.spin(explorer_navigator)
    explorer_navigator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()