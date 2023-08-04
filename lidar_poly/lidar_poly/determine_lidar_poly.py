import rclpy
from rclpy.node import Node

import math
import numpy as np

from std_msgs.msg import String, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, TransformStamped
from lidar_poly_interfaces.srv import PoseList
from lidar_poly_interfaces.msg import VehicleInfo

from lidar_poly.uf_support.support_functions import create_route_segments, get_point_on_route, find_closest_point, get_lidar_poly
#import lidar_poly.uf_support.support_functions
#import sys
#sys.path.append("lidar_poly/uf_support")
#import support_functions

#from support_functions import create_route_segments, get_point_on_route, find_closest_point, get_lidar_poly

D2R = math.pi/180.0
R2D = 180.0/math.pi

class route_pose_class:
  def __init__(self, pt=[0.0, 0.0, 0.0], heading_rad=0.0, state=1, w1_for_subsequent_segment=1.0, w2_for_subsequent_segment=1.0):
    self.pt = pt
    self.heading_rad = heading_rad
    self.state = state
    self.w1_for_subsequent_segment = w1_for_subsequent_segment
    self.w2_for_subsequent_segment = w2_for_subsequent_segment

class LidarStuffClass(Node):

    def __init__(self):
        super().__init__('determine_lidar_poly')
        self.cli = self.create_client(PoseList, 'provide_pose_list')

        # parameters
        self.declare_parameter("lidar_lane_width", 5.0) # m
        self.lidar_lane_width = self.get_parameter("lidar_lane_width").value
        self.declare_parameter("lidar_lane_length", 15.0) # m
        self.lidar_lane_length = self.get_parameter("lidar_lane_length").value
        self.declare_parameter("num_intervals", 10)
        self.num_intervals = self.get_parameter("num_intervals").value

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = PoseList.Request()

        self.subscription1 = self.create_subscription(
            VehicleInfo, 'vehicle_info_topic', self.listener1_callback, 10)
        self.subscription1  # prevent unused variable warning

        self.publisher_ = self.create_publisher(Marker, 'lidar_poly_points', 10)
        self.publisher2_ = self.create_publisher(Marker, 'path_points', 10)

        # class variables
        self.ready_to_process = False
        self.current_seg = 0   # current segment that the closest is on
        self.closest_point = np.array([0.0, 0.0])

    def listener1_callback(self, msg):
        # a new vehicle point has arrived
        self.get_logger().info('I heard: x = %f ; y = %f ; seg = %d' % (msg.veh_x, msg.veh_y, msg.current_seg))

        v_pt = np.array([msg.veh_x, msg.veh_y])
        my_seg = msg.current_seg

        if self.ready_to_process:  # this means we have a pose list
            closest_point, closest_seg_num, closest_uval, dist_so_far = find_closest_point(v_pt, my_seg, self.route_segs)

            lidar_poly = get_lidar_poly(self.lidar_lane_width, self.lidar_lane_length, closest_point, \
                  self.num_intervals, closest_seg_num, dist_so_far, self.route_segs)
            
            print('\nlidar_poly = ', lidar_poly, '\n')

            stamp = self.get_clock().now().to_msg()
            lidar_poly_marker = Marker()
            lidar_poly_marker.header.stamp = stamp
            lidar_poly_marker.header.frame_id = "utm_local"
            lidar_poly_marker.type = 4 # LINE_STRIP
            lidar_poly_marker.color = ColorRGBA(r=0.25, g=1.0, b=0.25, a=1.0)
            lidar_poly_marker.scale.x = 0.5
            lidar_poly_marker.id = 55

            for i in range(len(lidar_poly)):
                lidar_poly_marker.points.append(Point(x=lidar_poly[i][0], y=lidar_poly[i][1]))
            
            lidar_poly_marker.points.append(Point(x=lidar_poly[0][0], y=lidar_poly[0][1]))

            self.publisher_.publish(lidar_poly_marker)
            self.publisher2_.publish(self.path_line_marker)

    def send_request(self):  # no data is sent in the request
        self.future = self.cli.call_async(self.req)

def main(args=None):
    rclpy.init(args=args)

    lidar_obstacle = LidarStuffClass()
    lidar_obstacle.send_request()

    while rclpy.ok():
        rclpy.spin_once(lidar_obstacle)

        if lidar_obstacle.future.done():
            try:
                response = lidar_obstacle.future.result()
            except Exception as e:
                lidar_obstacle.get_logger().warn(
                    'Service call failed %r' % (e,))
            else:
                num_poses = response.num_poses

                lidar_obstacle.get_logger().info(
                    f"I received {num_poses} poses. ")

                # create the route_poses list
                     
                lidar_obstacle.route_pose_list = []
                my_pt = np.array([0.0, 0.0, 0.0])
                w1 = 1.0
                w2 = 1.0

                for i in range(num_poses):
                    my_pt[0] = response.pose_x[i]
                    my_pt[1] = response.pose_y[i] 
                    
                    my_heading_deg = response.heading_deg[i]
                    my_state = response.state[i]
                    temp_pose = route_pose_class(np.array([my_pt[0], my_pt[1], 0.0]), my_heading_deg*D2R, my_state, w1, w2)
                    lidar_obstacle.route_pose_list.append(temp_pose)
  
                want_loop = False
                dist_between_segments = 0.05 # m
                lidar_obstacle.route_segs = create_route_segments(lidar_obstacle.route_pose_list, want_loop, dist_between_segments)
                # pop off the last route segment (it was added to get an extension for control purposes)
                lidar_obstacle.route_segs.pop()

                lidar_obstacle.route_pose_list_u = []
                newpt_route_pose_list = np.zeros(3)

                for route_seg in lidar_obstacle.route_segs:
                   for u in np.arange(0.0, 1.0, 0.05):
                        newpt_route_pose_list = get_point_on_route(route_seg, u)
                        lidar_obstacle.route_pose_list_u.append(newpt_route_pose_list)
                
                stamp = lidar_obstacle.get_clock().now().to_msg()
                lidar_obstacle.path_line_marker = Marker()
                lidar_obstacle.path_line_marker.header.stamp = stamp
                lidar_obstacle.path_line_marker.header.frame_id = "utm_local"
                lidar_obstacle.path_line_marker.type = 4 # LINE_STRIP
                lidar_obstacle.path_line_marker.color = ColorRGBA(r=0.88, g=0.53, b=0.25, a=1.0)
                lidar_obstacle.path_line_marker.scale.x = 0.5
                lidar_obstacle.path_line_marker.id = 55
  
                for i in range(len(lidar_obstacle.route_pose_list_u)):
                    lidar_obstacle.path_line_marker.points.append(Point(x=lidar_obstacle.route_pose_list_u[i][0], y=lidar_obstacle.route_pose_list_u[i][1]))

                lidar_obstacle.publisher2_.publish(lidar_obstacle.path_line_marker)

                lidar_obstacle.ready_to_process = True
                lidar_obstacle.get_logger().info('Ready to proceed.')

            break

    rclpy.spin(lidar_obstacle)

    lidar_obstacle.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()