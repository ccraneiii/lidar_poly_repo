import sys
import yaml
import math
import numpy as np
import matplotlib.pyplot as plt

from python_support.route_support import *
from python_support.support_functions import *

debug_print = False

# -------------------------------------------------------------------

# open the yaml file that contains the paths

if len(sys.argv) != 2:
   print('useage:  python3 3_lidar_route_poly.py poses_file.yaml\n')
   exit()
# ---------------------------------------------------------------
# make sure the input file exists and can be opened 

pose_yaml_file  = './yaml/' + sys.argv[1]

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

if debug_print:
    for key, value in dictionary.items():
        print (key + " : " + str(value))
        
# will test on path exit_row_1_to_refill

# there are four vehicle locations to use for the testing

vehicle_pt = np.empty([7,2])
current_seg = np.empty(7, dtype=np.int8)

vehicle_pt[0][0] = 647506.94
vehicle_pt[0][1] = 3316175.27
current_seg[0] = 0

vehicle_pt[1][0] = 647507.66
vehicle_pt[1][1] = 3316177.08
current_seg[1] = 0

vehicle_pt[2][0] = 647507.60
vehicle_pt[2][1] = 3316187.02
current_seg[2] = 2

vehicle_pt[3][0] = 647495.00
vehicle_pt[3][1] = 3316190.00
current_seg[3] = 3

vehicle_pt[4][0] = 647386.89
vehicle_pt[4][1] = 3316197.15
current_seg[4] = 5

vehicle_pt[5][0] = 647388.27 
vehicle_pt[5][1] = 3316193.45
current_seg[5] = 7

vehicle_pt[6][0] = 647391.00 
vehicle_pt[6][1] = 3316192.00
current_seg[6] = 7

# ---------------------------------------------------
# set up the 'to refill' pose list
num_exit_row_1_to_refill_poses = len(dictionary['exit_row_1_to_refill'])

exit_row_1_to_refill_poses = []
my_pt = [0.0, 0.0]
w1 = 1.0
w2 = 1.0

if debug_print:
    print('num poses = ', num_exit_row_1_to_refill_poses)
for i in range(num_exit_row_1_to_refill_poses):
    my_pt = dictionary['exit_row_1_to_refill'][i]['pose']['pt']
    my_pt.append(0.0)
    my_pt_np = np.asarray(my_pt)
    my_heading = dictionary['exit_row_1_to_refill'][i]['pose']['heading_deg']
    my_state = dictionary['exit_row_1_to_refill'][i]['pose']['state']
    exit_row_1_to_refill_poses.append(route_pose_class(my_pt_np, my_heading*D2R, my_state, w1, w2))
if(debug_print):
    for pose in exit_row_1_to_refill_poses:
        print(pose.state, ' ', pose.heading_rad*R2D, ' ', pose.pt, ', ', pose.w1_for_subsequent_segment)

# create the route segments for the exit_row_1_to_refill path
want_loop = False
dist_between_segments = 0.05 # m
route_seg_exit_row_1_to_refill = create_route_segments_GITHUB_VERSION(exit_row_1_to_refill_poses, want_loop, dist_between_segments)

# pop off the last route segment (it was added to get an extension for control purposes)
route_seg_exit_row_1_to_refill.pop()

sweep_path_exit_row_1_to_refill_u = []
newpt_exit_row_1_to_refill = np.zeros(3)

for route_seg in route_seg_exit_row_1_to_refill:
   if(route_seg.state != state_dict['END_PLUS_ONE']):
      for u in np.arange(0.0, 1.0, 0.05):
         newpt_exit_row_1_to_refill = get_point_on_route(route_seg, u)
         sweep_path_exit_row_1_to_refill_u.append(newpt_exit_row_1_to_refill)

# ---------------------------------------------------
# ---------------------------------------------------

case = 0  # have 7 vehicle points to check
v_pt = vehicle_pt[case]
my_seg = current_seg[case]

lidar_lane_width  = 5.0  # m
lidar_lane_length = 20.0 # m

closest_point, closest_seg_num, closest_uval, dist_so_far = find_closest_point(v_pt, my_seg, route_seg_exit_row_1_to_refill)

#print('closest pt = ', closest_point, ' ; closest seg num = ', closest_seg_num, ' ; closest uval = ', closest_uval)
#print('dist so far = ', dist_so_far)
#print('v_pt = ', v_pt)

lidar_poly, lidar_poly_pts = get_lidar_poly_pts(lidar_lane_width, lidar_lane_length, closest_point, closest_seg_num, dist_so_far, route_seg_exit_row_1_to_refill)

# ---------------------------------------------------
# ---------------------------------------------------

# draw the exit_row_1_to_refill route poses
xr = [0.0, 0.0]  # allocate variables
yr = [0.0, 0.0]

for pose in exit_row_1_to_refill_poses:
    c1 = plt.Circle((pose.pt[0], pose.pt[1]), 0.3, color='black')
    xr[0] = pose.pt[0]
    yr[0] = pose.pt[1]
    xr[1] = xr[0] + 1.0*math.cos(pose.heading_rad)
    yr[1] = yr[0] + 1.0*math.sin(pose.heading_rad)
    plt.gca().add_artist(c1)
    plt.plot(xr,yr, color='turquoise')

# draw the exit_row_1_to_refill 'u' path

xs2 = []
ys2 = []
for pt in sweep_path_exit_row_1_to_refill_u:
   xs2.append(pt[0])
   ys2.append(pt[1])
	
plt.plot(xs2,ys2, color='orange')
  
# draw the vehicle point and the closest point
c1 = plt.Circle((closest_point[0], closest_point[1]), 0.2, color='blue')
plt.gca().add_artist(c1)
c2 = plt.Circle((v_pt[0], v_pt[1]), 0.2, color='red')
plt.gca().add_artist(c2)

# draw the lidar center points and left/right points
lidar_poly_shape = lidar_poly_pts.shape
num_in_lidar_poly_center =  lidar_poly_shape[0]
for i in range(num_in_lidar_poly_center):
    c3 = plt.Circle((lidar_poly_pts[i][0][0], lidar_poly_pts[i][0][1]), 0.15, color='blue')  # center points
    plt.gca().add_artist(c3)
    c4 = plt.Circle((lidar_poly_pts[i][2][0], lidar_poly_pts[i][2][1]), 0.15, color='green')  # left points
    plt.gca().add_artist(c4)
    c5 = plt.Circle((lidar_poly_pts[i][1][0], lidar_poly_pts[i][1][1]), 0.15, color='green')  # right points
    plt.gca().add_artist(c5)
    pass

'''
xl = []
yl = []
for i in range(num_in_lidar_poly_center):
    xl.append(lidar_poly_pts[i][1][0])
    yl.append(lidar_poly_pts[i][1][1])
for i in range(9, -1, -1):
    xl.append(lidar_poly_pts[i][2][0])
    yl.append(lidar_poly_pts[i][2][1])
xl.append(lidar_poly_pts[0][1][0])
yl.append(lidar_poly_pts[0][1][1])

plt.plot(xl,yl, color='darkviolet')
'''

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


