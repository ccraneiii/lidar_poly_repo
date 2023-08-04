import sys
import yaml
import math
import numpy as np
import matplotlib.pyplot as plt

from determine_lidar_poly import *

D2R = math.pi/180.0
R2D = 180.0/math.pi

# ---------------------------------------------------------------

if len(sys.argv) != 2:
   print('useage:  python3 main.py poses_file.yaml\n')
   exit()

# make sure the input file exists and can be opened 

pose_yaml_file  = sys.argv[1]

try:
    a_file = open(pose_yaml_file)
except IOError:
    print ("Could not open the file ", pose_yaml_file)
    exit()

stream = open(pose_yaml_file, 'r')
# ---------------------------------------------------------------
# load the yaml input file
dictionary = yaml.load(stream, Loader=yaml.Loader)
state_dict = dictionary.get('state_dict')

stream.close()

# set up the pose list
route_name = 'exit_row_1_to_refill'

num_poses = len(dictionary[route_name])

route_pose_list = []
my_pt = [0.0, 0.0]
w1 = 1.0
w2 = 1.0

for i in range(num_poses):
    my_pt = dictionary[route_name][i]['pose']['pt']
    my_pt.append(0.0)
    my_pt_np = np.asarray(my_pt)
    my_heading = dictionary[route_name][i]['pose']['heading_deg']
    my_state = dictionary[route_name][i]['pose']['state']
    route_pose_list.append(route_pose_class(my_pt_np, my_heading*D2R, my_state, w1, w2))

#############################################################
# The list of poses is created.
# Now create the route segments for the route_pose_list.
want_loop = False
dist_between_segments = 0.05 # m
route_segs = create_route_segments(route_pose_list, want_loop, dist_between_segments)

# pop off the last route segment (it was added to get an extension for control purposes)
route_segs.pop()

route_pose_list_u = []
newpt_route_pose_list = np.zeros(3)

for route_seg in route_segs:
   #if(route_seg.state != state_dict['END_PLUS_ONE']):
   for u in np.arange(0.0, 1.0, 0.05):
      newpt_route_pose_list = get_point_on_route(route_seg, u)
      route_pose_list_u.append(newpt_route_pose_list)

vehicle_pt = np.empty([7,2])
current_seg = np.empty(7, dtype=np.int8)

# The route segments are created. Ready to get closest point to vehicle 
#    point and then lidar polygon.

vehicle_pt[0][0] = 647506.94
vehicle_pt[0][1] = 3316175.27
current_seg[0] = 0

vehicle_pt[1][0] = 647507.66
vehicle_pt[1][1] = 3316177.08
current_seg[1] = 0

case = 0  # have 2 vehicle points to check
v_pt = vehicle_pt[case]
my_seg = current_seg[case]

lidar_lane_width  = 5.0  # m
lidar_lane_length = 20.0 # m
num_poly_intervals = 10

closest_point, closest_seg_num, closest_uval, dist_so_far = find_closest_point(v_pt, my_seg, route_segs)

lidar_poly = get_lidar_poly(lidar_lane_width, lidar_lane_length, closest_point, \
             num_poly_intervals, closest_seg_num, dist_so_far, route_segs)

#print('closest_point = ', closest_point, ' ; closest_seg_num = ', closest_seg_num)
#print('closest_uval = ', closest_uval, 'dist_so_far = ', dist_so_far)
#print('lidar poly = ', lidar_poly)

# draw the results
# draw the route poses
xr = [0.0, 0.0]  # allocate variables
yr = [0.0, 0.0]

for pose in route_pose_list:
    c1 = plt.Circle((pose.pt[0], pose.pt[1]), 0.3, color='black')
    xr[0] = pose.pt[0]
    yr[0] = pose.pt[1]
    xr[1] = xr[0] + 1.0*math.cos(pose.heading_rad)
    yr[1] = yr[0] + 1.0*math.sin(pose.heading_rad)
    plt.gca().add_artist(c1)
    plt.plot(xr,yr, color='turquoise')
    
# draw the route_pose_list 'u' path
xs2 = []
ys2 = []
for pt in route_pose_list_u:
   xs2.append(pt[0])
   ys2.append(pt[1])
plt.plot(xs2,ys2, color='orange')

# draw the vehicle point and the closest point
c1 = plt.Circle((closest_point[0], closest_point[1]), 0.2, color='blue')
plt.gca().add_artist(c1)
c2 = plt.Circle((v_pt[0], v_pt[1]), 0.2, color='red')
plt.gca().add_artist(c2)

# draw the lidar polygon
xl = []
yl = []
num_p = len(lidar_poly)
for i in range(num_p):
    xl.append(lidar_poly[i][0])
    yl.append(lidar_poly[i][1])
xl.append(lidar_poly[0][0])
yl.append(lidar_poly[0][1])
plt.plot(xl,yl, color='darkviolet')

plt.xlabel('UTM Easting (m)')
plt.ylabel('UTM Northing (m)')
plt.axis('equal')
plt.show()