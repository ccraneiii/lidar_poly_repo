import rclpy
from rclpy.node import Node

import os
import yaml
import math
import numpy as np

from ament_index_python.packages import get_package_share_directory

from lidar_poly_interfaces.srv import PoseList

D2R = math.pi/180.0
R2D = 180.0/math.pi

#- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
class route_pose_class:
  def __init__(self, pt=[0.0, 0.0, 0.0], heading_rad=0.0, state=1, w1_for_subsequent_segment=1.0, w2_for_subsequent_segment=1.0):
    self.pt = pt
    self.heading_rad = heading_rad
    self.state = state
    self.w1_for_subsequent_segment = w1_for_subsequent_segment
    self.w2_for_subsequent_segment = w2_for_subsequent_segment
#- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
class PoseListProvider(Node):

    def __init__(self):
        super().__init__('pose_list_service')
        self.srv = self.create_service(PoseList, 'provide_pose_list', self.provide_pose_list_callback)
        
    def provide_pose_list_callback(self, request, response):
        
        response.num_poses = len(self.route_pose_list)
        for i in range(len(self.route_pose_list)):
            response.pose_x[i] = self.route_pose_list[i].pt[0]
            response.pose_y[i] = self.route_pose_list[i].pt[1]
            response.heading_deg[i] = self.route_pose_list[i].heading_rad*R2D
            response.state[i] = self.route_pose_list[i].state
        
        self.get_logger().info('Incoming request.\n')
        
        return response
    
#- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

def main():

    rclpy.init()
    pose_list_provider = PoseListProvider()

    pose_yaml_file = get_package_share_directory('lidar_poly') + '/yaml/230731_pose_list_Tyndall.yaml'
    
    try:
        a_file = open(pose_yaml_file)
    except IOError: 
        print ("Could not open the file ", pose_yaml_file)
        exit()

    # load the yaml input file
    stream = open(pose_yaml_file, 'r') 
    dictionary = yaml.load(stream, Loader=yaml.Loader)
    state_dict = dictionary.get('state_dict')
    stream.close()

    # set up the pose list
    route_name = 'exit_row_1_to_refill'

    num_poses = len(dictionary[route_name])

    pose_list_provider.route_pose_list = []
    my_pt = [0.0, 0.0]
    w1 = 1.0
    w2 = 1.0

    for i in range(num_poses):
        my_pt = dictionary[route_name][i]['pose']['pt']
        my_pt.append(0.0)
        my_pt_np = np.asarray(my_pt)
        my_heading = dictionary[route_name][i]['pose']['heading_deg']
        my_state = dictionary[route_name][i]['pose']['state']
        pose_list_provider.route_pose_list.append(route_pose_class(my_pt_np, my_heading*D2R, my_state, w1, w2))

    rclpy.spin(pose_list_provider)

    rclpy.shutdown()


if __name__ == '__main__':
    main()