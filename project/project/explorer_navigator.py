import rclpy

from rclpy.node import Node
from project.srv import PathPlanner
from project.srv import GetBestNodeForExploration
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

class ExplorerNavigator(Node):
    GRID_TOPIC = ""
    POSE_TOPIC = ""

    def __init__(self):
        super().__init__('explorer_navigator')

        self.current_map = None
        self.robot_pose = None 
        self.current_grid_pose = None 
        self.current_exploration_goal_grid = None 
        self.current_navigation_path_grid = None
        self.current_navigation_goal_map = None 

        self.state = 'IDLE' 

        self.map_sub = self.create_subscription(OccupancyGrid, self.GRID_TOPIC, self.map_callback, 10)
        self.robot_pose_sub = self.create_subscription(OccupancyGrid, self.POSE_TOPIC, self.robot_pose_callback, 10)
        self.navigation_client = self.create_client(PathPlanner, 'navigate')
        self.exploration_client = self.create_client(GetBestNodeForExploration, 'explore')

        while not self.navigation_client.wait_for_service(timeout_sec=1.0) or not self.exploration_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Exploration and Navigiation not availabe, waiting')

        self.navigiation_request = PathPlanner().request
        self.exploration_request = GetBestNodeForExploration().request

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
        request = GetBestNodeForExploration.Request()
        request.map = self.current_map
        future = self.exploration_client.call_async(request)
        future.add_done_callback(self.exploration_response_callback)

    def exploration_response_callback(self, future):
        try:
            response = future.result()
            if response.best_node:
                self.current_exploration_goal_grid = (response.best_node[0], response.best_node[1])
                self.trigger_navigation()
            else:
                self.state = 'IDLE'
        except Exception as e:
            self.state = 'IDLE' 

    def trigger_navigation(self):
        if not self.current_map or not self.current_grid_pose or not self.current_exploration_goal_grid:
            self.state = 'IDLE'
            return

        self.state = 'NAVIGATING'
        
        request = PathPlanner.Request()
        request.map = self.current_map
        request.start = [self.current_grid_pose[0], self.current_grid_pose[1]]
        request.end = [self.current_exploration_goal_grid[0], self.current_exploration_goal_grid[1]]

        future = self.navigation_client.call_async(request)
        future.add_done_callback(self.navigation_response_callback)

    def navigation_response_callback(self, future):
        try:
            response = future.result()
            if response.path:
                self.current_navigation_path_grid = [(response.path[i], response.path[i+1]) for i in range(0, len(response.path), 2)]
                self.send_path_to_robot_driver(self.current_navigation_path_grid)

            else:
                self.state = 'IDLE' 
                self.trigger_exploration() 
        except Exception as e:
            self.state = 'IDLE'
            self.trigger_exploration() 

def main(args=None):
    rclpy.init(args=args)
    explorer_navigator = ExplorerNavigator()
    rclpy.spin(explorer_navigator)
    explorer_navigator.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
