import rclpy
from rclpy.node import Node

import numpy as np

from lidar_poly_interfaces.msg import VehicleInfo

class  VehicleInfoPublisher(Node):

    def __init__(self):
        super().__init__('vehicle_info_publisher')
        self.publisher_ = self.create_publisher(VehicleInfo, 'vehicle_info_topic', 10)
        timer_period = 2.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        # there are seven vehicle locations to use for the testing

        self.vehicle_pt = np.empty([7,2])
        self.current_seg = np.empty(7, dtype=np.int8)

        self.vehicle_pt[0][0] = 647506.94
        self.vehicle_pt[0][1] = 3316175.27
        self.current_seg[0] = 0

        self.vehicle_pt[1][0] = 647507.66
        self.vehicle_pt[1][1] = 3316177.08
        self.current_seg[1] = 0

        self.vehicle_pt[2][0] = 647507.60
        self.vehicle_pt[2][1] = 3316187.02
        self.current_seg[2] = 2

        self.vehicle_pt[3][0] = 647495.00
        self.vehicle_pt[3][1] = 3316190.00
        self.current_seg[3] = 3

        self.vehicle_pt[4][0] = 647386.89
        self.vehicle_pt[4][1] = 3316197.15
        self.current_seg[4] = 5

        self.vehicle_pt[5][0] = 647388.27 
        self.vehicle_pt[5][1] = 3316193.45
        self.current_seg[5] = 7

        self.vehicle_pt[6][0] = 647391.00 
        self.vehicle_pt[6][1] = 3316192.00
        self.current_seg[6] = 7

    def timer_callback(self):
        msg = VehicleInfo()
        case_num = self.i % len(self.current_seg)
        msg.veh_x = self.vehicle_pt[case_num][0]
        msg.veh_y = self.vehicle_pt[case_num][1]
        msg.current_seg = int(self.current_seg[case_num])
        
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: %d' % self.i)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)

    vehicle_info_publisher = VehicleInfoPublisher()

    rclpy.spin(vehicle_info_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    vehicle_info_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()