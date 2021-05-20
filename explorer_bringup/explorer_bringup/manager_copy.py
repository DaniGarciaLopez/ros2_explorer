from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from explorer_interfaces.action import Wander
from nav2_msgs.action import NavigateToPose

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

#ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {stamp: {sec: 0}, frame_id: 'map'}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"


class WandererClient(Node):

    def __init__(self):
        super().__init__('wanderer_client')
        self._action_client = ActionClient(self, Wander, 'wander')
        self.navigation_client = NavigationClient()

    def feedback_callback(self, feedback):
        self.get_logger().info('Received feedback: {0}'.format(feedback.feedback.sequence))          

    def send_goal(self):
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        goal_msg = Wander.Goal()
        goal_msg.strategy= 1
        goal_msg.map_completed_thres = 0.2

        self.get_logger().info('Sending exploration goal request...')

        self._send_goal_future = self._action_client.send_goal_async(goal_msg,feedback_callback=self.feedback_callback)

        rclpy.spin_until_future_complete(self, self._send_goal_future)

        goal_handle = self._send_goal_future.result()

        get_result_future = self._action_client._get_result_async(goal_handle)

        rclpy.spin_until_future_complete(self, get_result_future)
    
class NavigationClient(Node):

    def __init__(self):
        super().__init__('navigation_client')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Exploration goal rejected')
            return

        self.get_logger().info('Navigation goal accepted')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Arrived to home position')
        else:
            self.get_logger().info('Goal failed with status: {0}'.format(status))
            
    def send_goal(self):
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.pose.orientation.w=1.0 #Home position

        self.get_logger().info('Sending navigation goal request...')

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)

        self._send_goal_future.add_done_callback(self.goal_response_callback)





def main(args=None):
    rclpy.init(args=args)

    wanderer_client = WandererClient()

    wanderer_client.send_goal()

    rclpy.spin(wanderer_client)


if __name__ == '__main__':
    main()