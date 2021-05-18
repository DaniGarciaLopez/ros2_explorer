import rclpy
import numpy
import os
import csv
import pandas as pd
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import ParameterType



class Subscriber(Node):

    def __init__(self):
        super().__init__('watchtower')
        # Define subscriber
        self.subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.listener_callback,
            10)
        # Define publisher
        self.publisher_ = self.create_publisher(Float32, 'map_progress', 10)
        self.free_thresh = 0.65
        # Declare map_name parameter
        self.declare_parameter('map_name')
        map_param = self.get_parameter('map_name') 
        self.get_logger().info('Map selected = %s' % (str(map_param.value),))
        # Read map file
        package_share_directory = get_package_share_directory('explorer_gazebo')
        map_folder_directory = os.path.join(package_share_directory, 'maps')
        map_file = os.path.join(map_folder_directory, map_param.value + '.csv')
        df=pd.read_csv(map_file, sep=',',header=None)
        sim_map_array = df.values
        print(sim_map_array)
        self.free_space = numpy.count_nonzero(sim_map_array == 0)  
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        map_array = numpy.asarray(msg.data)
        resolution = msg.info.resolution
        map_explored = numpy.count_nonzero(map_array <= self.free_thresh) * resolution**2
        percentage_explored = map_explored/self.free_space
        map_explored_msg = Float32()
        map_explored_msg.data = percentage_explored
        self.publisher_.publish(map_explored_msg)
        self.get_logger().info('Percentage explored = %s' % map_explored_msg.data)

def main(args=None):
    rclpy.init(args=args)

    watchtower = Subscriber()

    rclpy.spin(watchtower)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    watchtower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()