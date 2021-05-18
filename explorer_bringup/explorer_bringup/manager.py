from action_msgs.msg import GoalStatus
from explorer_interfaces.action import Wander

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

#ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {stamp: {sec: 0}, frame_id: 'map'}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"


class WandererClient(Node):

    def __init__(self):
        super().__init__('manager')
        self._action_client = ActionClient(self, Wander, 'wander')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback):
        self.get_logger().info('Received feedback: {0}'.format(feedback.feedback.sequence))

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal succeeded!')
        else:
            self.get_logger().info('Goal failed with status: {0}'.format(status))
            

    def send_goal(self):
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        goal_msg = Wander.Goal()
        goal_msg.strategy= 1
        goal_msg.map_completed_thres = 0.95

        self.get_logger().info('Sending goal request...')

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
class NavigationClient(Node):

    def __init__(self):
        super().__init__('manager')
        self._action_client = ActionClient(self, Wander, 'wander')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal succeeded!')
        else:
            self.get_logger().info('Goal failed with status: {0}'.format(status))
            
    def send_goal(self):
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        goal_msg = Wander.Goal()
        goal_msg.strategy= 1
        goal_msg.map_completed_thres = 0.6

        self.get_logger().info('Sending goal request...')

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)

        self._send_goal_future.add_done_callback(self.goal_response_callback)





def main(args=None):
    rclpy.init(args=args)

    wanderer_client = WandererClient()

    wanderer_client.send_goal()

    rclpy.spin(wanderer_client)


if __name__ == '__main__':
    main()