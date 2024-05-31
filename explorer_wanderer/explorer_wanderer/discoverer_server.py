# written by Enrique Fernández-Laguilhoat Sánchez-Biezma and Daniel García López

#!/usr/bin/env python3
# -*- coding: utf-8 -*-


# 3rd party
import numpy as np
import pandas as pd
import os
import csv
from ament_index_python.packages import get_package_share_directory

# ros
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from rclpy.action import ActionClient
from rcl_interfaces.msg import ParameterType
from action_msgs.msg import GoalStatus

# messages
from std_msgs.msg import Float32
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from nav2_msgs.action import NavigateToPose
from explorer_interfaces.action import Discover


# ros2 action send_goal wander explorer_interfaces/action/Wander "{strategy: 1, map_completed_thres: 0.6}"


class DiscovererServer(Node):
    def __init__(self):
        super().__init__('discoverer_server')
        self._action_server = ActionServer(self, Discover, 'discover', self.execute_callback)
        self.watchtower_subscription = self.create_subscription(Float32, 'map_progress', self.watchtower_callback, 10)
        self.watchtower_subscription  # prevent unused variable warning
        self.navigation_client = NavigationClient()
        self.stop_discovering = False
        self.map_completed_thres=1.0 #Initialize threshold to max (100%)
        self.get_logger().info("Discoverer Server is ready")

    def watchtower_callback(self, msg):
        # If map_progress is higher than the threshold send stop wandering signal
        if msg.data > self.map_completed_thres:
            self.stop_discovering = True

    def execute_callback(self, goal_handle):
        self.get_logger().info("Discoverer Server received a goal")
        self.map_completed_thres=goal_handle.request.map_completed_thres
        self.get_logger().info("Map completed threshold set to: %s" %self.map_completed_thres)
        while not self.stop_discovering:
            self.navigation_client.send_goal()

        self.get_logger().info('Discovering Finished')
        goal_handle.succeed()
        return Discover.Result()


class NavigationClient(Node):
    def __init__(self):
        super().__init__('navigation_client')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.cartographer = CartographerSubscriber()  # a cartographer subscription is created to access the occupancy
        rclpy.spin_once(self.cartographer)
        # grid and determine which positions to navigate to

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
            self.get_logger().info('Arrived at destination')
        else:
            self.get_logger().info('Goal failed with status: {0}'.format(status))

        rclpy.spin_once(self.cartographer)

    def send_goal(self):
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        rclpy.spin_once(self.cartographer)  # refresh the list of accessible waypoints
        waypoint = self.cartographer.sorted_accessible_waypoints[0]  # grab the first waypoint
        self.cartographer.sorted_accessible_waypoints = self.cartographer.sorted_accessible_waypoints[1:]  # pop the
        # first element from the list, in case the
        # accessible waypoints didn't refresh

        # write command
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'base_footprint'
        goal_msg.pose.pose.position.x = float(waypoint[0])
        goal_msg.pose.pose.position.y = float(waypoint[1])
        # goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info(
            'Sending navigation goal request x: ' + str(round(goal_msg.pose.pose.position.x, 2)) + ' y: ' + str(
                round(goal_msg.pose.pose.position.y, 2)))

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

        rclpy.spin_until_future_complete(self, self._send_goal_future)

        goal_handle = self._send_goal_future.result()
        get_result_future = goal_handle.get_result_async()

        rclpy.spin_until_future_complete(self, get_result_future)


class CartographerSubscriber(Node):
    def __init__(self):
        super().__init__('cartographer_subscriber')
        self.occupancy_subscription = self.create_subscription(OccupancyGrid, 'map', self.occupancy_callback, 10)

        self.waypoints = self.generate_list_of_waypoints(n_of_waypoints=100, step=0.2)
        self.accessible_waypoints = np.array([])
        self.sorted_accessible_waypoints = np.array([])
        self.occupancy_value = np.array([])

    def occupancy_callback(self, msg):
        """

        The cartographer subscriber callback function refreshes the list of accessible waypoints. It sorts them and
        saves them in the self.sorted_accessible_waypoints variable.

        :param msg: OccupancyGrid message. Includes map metadata and an array with the occupancy probability values
        :return: None
        """

        data = np.array(msg.data)  # download the occupancy grid
        current_map_width = msg.info.width  # get the current map width
        current_map_height = msg.info.height  # get the current map height
        resolution = msg.info.resolution  # get the resolution

        # reshape the data so it resembles the map shape
        data = np.reshape(data, (current_map_height, current_map_width))

        # Here we go through every waypoint and save the ones that are accessible.
        # An accessible waypoint is one which has no obstacles, and has few or no unknown squares in the vicinity.
        self.accessible_waypoints = np.array([])
        self.occupancy_value = np.array([])
        for waypoint in self.waypoints:
            try:
                occupancy_grid_coordinates = [int((waypoint[1] + 2.3) / resolution), int((waypoint[0] + 2.3) /
                                                                                         resolution)]
                conv = self.convolute(data, occupancy_grid_coordinates, size=9)  # perform convolution

                # if the convolution returns True, it means the WP is accessible, so it is stored in
                # self.accessible_waypoints
                if conv[0]:
                    self.accessible_waypoints = np.append(self.accessible_waypoints, waypoint)
                    self.occupancy_value = np.append(self.occupancy_value, conv[1])
            # because the waypoint array is over-sized, we need to remove the values that are out of range
            except IndexError:
                pass

        # reshape the accessible waypoints array to shape (n, 2)
        self.accessible_waypoints = self.accessible_waypoints.reshape((-1, 2))

        # Sorting waypoints according to occupancy value. This allows the robot to prioritize the waypoints with
        # more uncertainty (it wont access the areas that are completely clear, thus going to the discovery frontier)
        occupancy_value_idxs = self.occupancy_value.argsort()
        self.sorted_accessible_waypoints = self.accessible_waypoints[occupancy_value_idxs[::-1]]

        # At the beginning, when all values are uncertain, we add some hardcoded waypoints so it begins to navigate
        # and has time to discover accessible areas
        if np.size(self.sorted_accessible_waypoints) == 0:
            self.sorted_accessible_waypoints = np.array([[1.5, 0.0], [0.0, 1.5], [-1.5, 0.0], [0.0, -1.5]])

        # Once we have the new waypoints, they are saved in self.sorted_accessible_waypoints for use by the Navigator
        # client
        self.get_logger().info('Accessible waypoints have been updated...')

    @staticmethod
    def convolute(data, coordinates, size=3, threshold=40):
        """
        This function calculates the average occupancy probability at 'coordinates' for an area of size (size x size)
        around said point.

        :param data: Occupancy Grid Data (shaped to (x, y) map dimensions)
        :param coordinates: the coordinates of the OccupancyGrid to convolute around
        :param size: size of the kernel
        :param threshold: threshold of accessibility
        :return: True or False, depending on whether the waypoint is accessible or not.
        :return: average: average occupancy probability of the convolution
        """
        sum = 0
        for x in range(int(coordinates[0] - size / 2), int(coordinates[0] + size / 2)):
            for y in range(int(coordinates[1] - size / 2), int(coordinates[1] + size / 2)):
                # if the area is unknown, we add 100 to sum.
                if data[x, y] == -1:
                    sum += 100
                # if occupancy state is above 50 (occupied), we add 1M to the sum so that the robot DOES NOT
                # access areas near walls.
                elif data[x, y] > 50:
                    sum += 1000000
                # if the occupancy state is below 50 and known, just add the value to sum.
                else:
                    sum += data[x, y]

        # average value for the square is computed
        average = sum / (size * size)
        if average < threshold:
            # if the average of the squares is below the threshold, the waypoint is accessible
            return True, average
        else:
            # if the average is above the threshold, the waypoint has either too many unknowns, or an obstacle
            return False, average

    def generate_list_of_waypoints(self, n_of_waypoints, step):
        """

        Generates a grid of waypoints of size ('n_of_waypoints' * 'n_of_waypoints') and step size 'step'

        :param n_of_waypoints: number of total waypoints to generate per side
        :param step: float resolution of the waypoints
        :return waypoints: 2D numpy array of a list of coordinates of size dim x 2,
        where dim is the number of waypoints
        """

        waypoints = np.zeros((n_of_waypoints * n_of_waypoints, 2))

        i = 0
        for index_y in range(n_of_waypoints):
            for index_x in range(n_of_waypoints):
                waypoints[i] = [float(index_x) / (1/step), float(index_y) / (1/step)]
                i += 1

        self.get_logger().info("Grid of waypoints has been generated.")
        return waypoints


def main(args=None):
    rclpy.init(args=args)

    discoverer_server = DiscovererServer()

    rclpy.spin(discoverer_server)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
