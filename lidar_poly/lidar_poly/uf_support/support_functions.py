import numpy as np
import math
from enum import Enum

class myState(Enum):
    TRACTOR_OFF = 0
    TRACTOR_ON = 1
    TRACTOR_OUTSIDE = 2
    TRACTOR_ENTRY_EXTENSION_PT = 3
    TRACTOR_EXIT_EXTENSION_PT = 4
    TRACTOR_ENTRY_TURN_PT = 5
    TRACTOR_EXIT_TURN_PT = 6
    TRACTOR_START = 7
    TRACTOR_END = 8
    TRACTOR_UTURN_PT1 = 9
    TRACTOR_UTURN_PT2 = 10
    TRACTOR_UTURN_PT3 = 11
    TRACTOR_CORNER = 12
    NOZZLE_ON = 13
    NOZZLE_OFF = 14
    EXTRA_EDGE = 15
    EDGE1_TO_REFILL_PT1 = 16
    EDGE1_TO_REFILL_MID_PT = 17
    PRE_REFILL_PT = 18
    POST_REFILL_PT = 19
    EDGE3_RETURN_PT = 20
    EDGE1_RETURN_PT = 21
    END_PLUS_ONE = 22
    EXIT_LEFT = 23
    FAR_EDGE_EXIT = 24
    ASTAR_POINT = 25
    PRE_PRE_REFILL_PT= 26
    PRE_REFILL_U_TURN_PT= 27
    PRE_ROW_ENTRY_PT= 28
    REFILL_PT= 29

class route_pose_class:
  def __init__(self, pt=[0.0, 0.0, 0.0], heading_rad=0.0, state=1, w1_for_subsequent_segment=1.0, w2_for_subsequent_segment=1.0):
    self.pt = pt
    self.heading_rad = heading_rad
    self.state = state
    self.w1_for_subsequent_segment = w1_for_subsequent_segment
    self.w2_for_subsequent_segment = w2_for_subsequent_segment

class route_segment_class:
  def __init__(self, p0, p1, p2, p3, w1, w2, state, dist_between_pts):
    self.p0 = p0
    self.p1 = p1
    self.p2 = p2
    self.p3 = p3
    self.w1 = w1
    self.w2 = w2
    self.state = state

    if dist_between_pts > 0.0:
      self.length = self.get_length()  # meters
      self.dist_between_pts = dist_between_pts  # meters
      self.calculate_route_segment_points(dist_between_pts)
      # the last function calculates an np array named 'pt_info' of size n x 4 of
      #    evenly spaced points along the path [uval, x, y, dist_from_p0]

  def get_length(self):
    length = 0.0
    last_point = np.copy(self.p0)
    
    for u in np.arange(0.0, 1.001, 0.001):
      denom = (1.0 - u) * (1.0 - u) * (1.0 - u) + 3.0 * u * (1.0 - u) * (1.0 - u) * self.w1 + 3.0 * u * u * (1.0 - u) * self.w2 + u * u * u;
      current_pt = ((1.0 - u) * (1.0 - u) * (1.0 - u) * self.p0 + 3.0 * u * (1.0 - u) * (1.0 - u) * self.w1 * self.p1 \
                  + 3.0 * u * u * (1.0 - u) * self.w2 * self.p2 + u * u * u * self.p3) / denom;
      length += np.linalg.norm(current_pt - last_point)
      last_point = current_pt

    return length
  
  def get_point(self, u):
    denom = (1.0 - u) * (1.0 - u) * (1.0 - u) + 3.0 * u * (1.0 - u) * (1.0 - u) * self.w1 + 3.0 * u * u * (1.0 - u) * self.w2 + u * u * u;
    pt = ((1.0 - u) * (1.0 - u) * (1.0 - u) * self.p0 + 3.0 * u * (1.0 - u) * (1.0 - u) * self.w1 * self.p1 + 3.0 * u * u * (1.0 - u) * self.w2 * self.p2 + u * u * u * self.p3) / denom
    return pt  
  
  def get_heading_rad(self,u):
    P0 = self.p0
    P1 = self.p1
    P2 = self.p2
    P3 = self.p3
    w1 = self.w1
    w2 = self.w2

    pyNumer = (-9 * P1 * w1 * w2 + 9 * P2 * w1 * w2 - 3 * w1 * P0 + 6 * P0 * w2 + 9 * w1 * P1 - 9 * P2 * w2 - 6 * P3 * w1 + 3 * P3 * w2 - 3 * P0 + 3 * P3) * u ** 4 \
          + (18 * P1 * w1 * w2 - 18 * P2 * w1 * w2 + 12 * w1 * P0 - 18 * P0 * w2 - 18 * w1 * P1 + 18 * P2 * w2 + 6 * P3 * w1 + 6 * P0 - 6 * P3) * u ** 3 \
          + (-9 * P1 * w1 * w2 + 9 * P2 * w1 * w2 - 18 * w1 * P0 + 18 * P0 * w2 + 18 * w1 * P1 - 18 * P2 * w2 - 3 * P0 + 3 * P3) * u ** 2 \
          + (12 * w1 * P0 - 6 * P0 * w2 - 12 * w1 * P1 + 6 * P2 * w2) * u \
          - 3 * w1 * P0 + 3 * w1 * P1


    pyDenom = 1 + (3 * w1 - 3 * w2) ** 2 * u ** 6 + 2 * (-6 * w1 + 3 * w2 + 3) * (3 * w1 - 3 * w2) * u ** 5 \
          + (2 * (3 * w1 - 3) * (3 * w1 - 3 * w2) + (-6 * w1 + 3 * w2 + 3) ** 2) * u ** 4 \
          + (6 * w1 - 6 * w2 + 2 * (3 * w1 - 3) * (-6 * w1 + 3 * w2 + 3)) * u ** 3 \
          + (-12 * w1 + 6 * w2 + 6 + (3 * w1 - 3) ** 2) * u ** 2 \
          + (6 * w1 - 6) * u

    dxdu = pyNumer[0]/pyDenom
    dydu = pyNumer[1]/pyDenom

    return (math.atan2(dydu, dxdu))
  
  def get_u(self, dist):
    uval = 0.0
    mydist = 0.000000001  # did this in case input 'dist' is zero and then will output 'uval' equals zero
    last_pt = np.copy(self.p0)

    while(mydist < dist and uval <= 1.0):
      uval += 0.001
      pt = np.copy(self.get_point(uval))
      mydist += np.linalg.norm(pt-last_pt)
      last_pt = np.copy(pt)
      
    return uval

  def calculate_route_segment_points(self, dist_between_pts):
    if dist_between_pts < 0.0:
      return np.array([0.0,0.0,0.0,0.0])  # return junk
    
    route_data = np.zeros((5001, 5), dtype = float)  #  u, x, y, dist

    uval = 0.0
    dist_so_far = 0.0
    prev_pt = np.copy(self.p0)
    for i in np.arange(5001):
      pt = self.get_point(uval)
      route_data[i, 0] = uval
      route_data[i, 1] = pt[0]
      route_data[i, 2] = pt[1]
      route_data[i, 3] = 0.0  # very slow ; will calculate only for evenly spaced points
      dist_so_far += np.linalg.norm(pt-prev_pt)
      route_data[i, 4] = dist_so_far
      prev_pt = np.copy(pt)
      uval += 0.0002

    numpts = int(self.length/dist_between_pts) + 1
    self.pt_info = np.zeros((numpts,5))
     
    find_dist = 0.0
    dist_so_far = 0.0
    search_index = 0

    for i in np.arange(numpts):

      if find_dist >= self.length:
        now_u = 1.0
        self.pt_info[i,0] = route_data[5000, 0]
        self.pt_info[i,1] = route_data[5000, 1]
        self.pt_info[i,2] = route_data[5000, 2]
        self.pt_info[i,3] = route_data[5000, 3]
        self.pt_info[i,4] = route_data[5000, 4]
      elif np.isclose(find_dist, 0.0, atol=0.001):
        now_u = 0.0
        self.pt_info[i,0] = route_data[0, 0]
        self.pt_info[i,1] = route_data[0, 1]
        self.pt_info[i,2] = route_data[0, 2]
        self.pt_info[i,3] = route_data[0, 3]
        self.pt_info[i,4] = route_data[0, 4]
      else:
        while (dist_so_far < find_dist and search_index<5000):
          search_index += 1
          dist_so_far = route_data[search_index,4]
        self.pt_info[i,0] = route_data[search_index, 0]
        self.pt_info[i,1] = route_data[search_index, 1]
        self.pt_info[i,2] = route_data[search_index, 2]
        self.pt_info[i,3] = route_data[search_index, 3]
        self.pt_info[i,4] = route_data[search_index, 4]

      find_dist += dist_between_pts
        
# End of definition of route_segment_class

#----------------------------------------------------------------------------------
def get_u_value_for_dist(route_segment:route_segment_class, \
                          dist:float \
                         )->float:
  #
  # inputs -
  #     route_segment: a route_segment_struct
  #     dist: distance from point p0 of segment to the point designated by u
  # return - uval: the value of u on the path segment, 0 <= u <= 1, corresponding to dist
  #

  uval = 0.0
  mydist = 0.000000001  # did this in case input 'dist' is zero and then will output 'uval' equals zero
  last_pt = route_segment.p0

  while(mydist < dist and uval <= 1.0):
    uval += 0.001
    pt = get_point_on_route(route_segment, uval)
    mydist += np.linalg.norm(pt-last_pt)
    last_pt = pt

  return uval

#----------------------------------------------------------------------------------
def get_point_on_route(route_segment:route_segment_class, \
                       u:float \
                      )-> np.array:
  #
  # inputs -
  #    route_segment: a route_segment
  #    u: the value of u along the route segment, 0 <= u <= 1
  # outputs -
  #    pt: the x,y,z coordinates of the point on the path

  denom = (1.0 - u) * (1.0 - u) * (1.0 - u) + 3.0 * u * (1.0 - u) * (1.0 - u) * route_segment.w1 + 3.0 * u * u * (1.0 - u) * route_segment.w2 + u * u * u;
  
  pt = ((1.0 - u) * (1.0 - u) * (1.0 - u) * route_segment.p0 + 3.0 * u * (1.0 - u) * (1.0 - u) * route_segment.w1 * route_segment.p1 + 3.0 * u * u * (1.0 - u) * route_segment.w2 * route_segment.p2 + u * u * u * route_segment.p3) / denom

  return pt
  
#----------------------------------------------------------------------------------
def create_route_segments(route_poses, want_loop, dist_between_pts):
  # inputs - route_poses: an array of 'route_pose_class'
  #
  # outputs- route_segments:  an array of 'route_segment_class'
  #          Note that there will ultimately be the same number of route_segments as there were route_poses.
  #          Either a stop extension segment will be added or a segment will be added to create the loop.
  
  route_segments = []
  
  num_points = len(route_poses)
  
  for i in range(num_points-1):  # there is one less route segment than there are route points (right now)
    p0 = route_poses[i].pt
    p3 = route_poses[i+1].pt
    
    w1 = route_poses[i].w1_for_subsequent_segment
    w2 = route_poses[i].w2_for_subsequent_segment
    
    L1L2dist = np.linalg.norm(p3-p0) / 4.0  # the route segment control points are 1/4 the distance between the ends (p0 & p3)
    if L1L2dist > 20.0:
      L1L2dist = 20.0
    
    S1 = np.array([math.cos(route_poses[i].heading_rad), math.sin(route_poses[i].heading_rad), 0.0])
    S2 = np.array([math.cos(route_poses[i+1].heading_rad), math.sin(route_poses[i+1].heading_rad), 0.0])
    
    p1 = p0 + L1L2dist * S1
    p2 = p3 - L1L2dist * S2
    
    # now get the length of the segment
    length = 0.0
    last_point = p0
    
    for u in np.arange(0.0, 1.001, 0.001):
      denom = (1.0 - u) * (1.0 - u) * (1.0 - u) + 3.0 * u * (1.0 - u) * (1.0 - u) * w1 + 3.0 * u * u * (1.0 - u) * w2 + u * u * u;
      current_pt = ((1.0 - u) * (1.0 - u) * (1.0 - u) * p0 + 3.0 * u * (1.0 - u) * (1.0 - u) * w1 * p1 \
                  + 3.0 * u * u * (1.0 - u) * w2 * p2 + u * u * u * p3) / denom;
      length += np.linalg.norm(current_pt - last_point)
      last_point = current_pt
    
    route_segments.append(route_segment_class(p0, p1, p2, p3, w1, w2, route_poses[i].state, dist_between_pts))
    route_segments[-1].length = length
 
  if(want_loop):
    # add a route segment from the last route point to the initial route point
    p0 = p3
    p3 = route_poses[0].pt
    w1 = route_poses[num_points-1].w1_for_subsequent_segment
    w2 = route_poses[num_points-1].w2_for_subsequent_segment
    S1 = np.array([math.cos(route_poses[num_points-1].heading_rad), math.sin(route_poses[num_points-1].heading_rad), 0.0])
    S2 = np.array([math.cos(route_poses[0].heading_rad), math.sin(route_poses[0].heading_rad), 0.0])
    L1L2dist = np.linalg.norm(p3-p0) / 4.0  # the route segment control points are 1/4 the distance between the ends (p0 & p3)
    if L1L2dist > 20.0:
      L1L2dist = 20.0
    p1 = p0 + L1L2dist * S1
    p2 = p3 - L1L2dist * S2
    
    route_segments.append(route_segment_class(p0, p1, p2, p3, w1, w2, route_poses[num_points-1].state, dist_between_pts))
    
  else:  # no loop; add a stop segment (make sure the distance (20 here) is larger than the look-ahead-distance
    S2 = np.array([math.cos(route_poses[-1].heading_rad), math.sin(route_poses[-1].heading_rad), 0.0])
    p0 = p3
    p1 = p0 + 5.0 * S2
    p2 = p0 + 15.0 * S2
    p3 = p0 + 20.0 * S2
    w1 = 1.0
    w2 = 1.0
    #length = 20.0
    #state = uf_dict["END_PLUS_ONE"]
    state = myState['END_PLUS_ONE'].value

    route_segments.append(route_segment_class(p0, p1, p2, p3, w1, w2, state, dist_between_pts))

  return route_segments

#----------------------------------------------------------------------------------
def find_closest_point(v_pt, current_seg_num, route_segments):

  num_segments = len(route_segments)
  
  if len(v_pt) == 2:
     vehicle_pt = np.array([v_pt[0], v_pt[1], 0.0])

  if(current_seg_num > num_segments):
    print('The current segment number is ', current_seg_num,', but there are only ', num_segments, ' route segments.')
    return [0, 0, 0]
  
  closest_dist_current_seg = 99999.0
  closest_dist_next_seg    = 99999.0
  best_current_seg_index = 0
  best_next_seg_index = 0

  if (current_seg_num < num_segments):
    for current_seg_index in np.arange(len(route_segments[current_seg_num].pt_info)):
      pt_now = np.array([route_segments[current_seg_num].pt_info[current_seg_index, 1], route_segments[current_seg_num].pt_info[current_seg_index, 2], 0.0])
      dist_now = np.linalg.norm(pt_now - vehicle_pt)

      if dist_now < closest_dist_current_seg:
        closest_dist_current_seg = dist_now
        best_current_seg_index = current_seg_index

    for current_seg_index in np.arange(len(route_segments[(current_seg_num+1)%num_segments].pt_info)):
      pt_now = np.array([route_segments[(current_seg_num+1)%num_segments].pt_info[current_seg_index, 1], route_segments[(current_seg_num+1)%num_segments].pt_info[current_seg_index, 2], 0.0])
      dist_now = np.linalg.norm(pt_now - vehicle_pt)

      if dist_now < closest_dist_next_seg:
        closest_dist_next_seg = dist_now
        best_next_seg_index = current_seg_index

    if closest_dist_current_seg < closest_dist_next_seg:
      veh_seg_num = current_seg_num
      my_pt_info_index = best_current_seg_index
    else:
      veh_seg_num = (current_seg_num+1)%num_segments
      my_pt_info_index = best_next_seg_index
    
    my_closest_pt = np.array([route_segments[veh_seg_num].pt_info[my_pt_info_index,1], \
                              route_segments[veh_seg_num].pt_info[my_pt_info_index,2], 0.0])
    #closest_heading_rad = route_segments[veh_seg_num].pt_info[my_pt_info_index,3]
    
    #closest_pose = route_pose_class(my_closest_pt, \
    #                                closest_heading_rad, \
    #                                route_segments[veh_seg_num].state, \
    #                                route_segments[veh_seg_num].w1, \
    #                                route_segments[veh_seg_num].w2)
    
    my_uval = route_segments[veh_seg_num].pt_info[my_pt_info_index,0]
    dist_so_far = route_segments[veh_seg_num].pt_info[my_pt_info_index,4]
    
    if len(v_pt) == 2:
        return my_closest_pt[0:2], veh_seg_num, my_uval, dist_so_far
    else:
        return my_closest_pt, veh_seg_num, my_uval, dist_so_far
#----------------------------------------------------------------------------------
def get_lidar_poly(lidar_lane_width, lidar_lane_length, closest_point, num_poly_intervals, \
                   closest_seg_num, dist_so_far, route_segs):
    # There will be one more polygon point (on each side of polygon) than interval.
    # For example, if the lidar_lane_length is 20 m and the num_poly_intervals is 10,
    #    then there will be 11 points on each side of the lidar polygon, spaced 2 m apart
    
    num_poly_pts = num_poly_intervals + 1  # there will be num_poly_pts-1 intervals
    total_seg_num = len(route_segs)
    
    shorty_case = False
    
    num_of_additional_segs_needed = 0
    poly_length_so_far = 0.0
    dist_left_to_go = lidar_lane_length
    
    length_of_closest_seg = route_segs[closest_seg_num].length
    if lidar_lane_length > length_of_closest_seg - dist_so_far:  # will need at least 1 more segment
        poly_length_so_far = length_of_closest_seg - dist_so_far
        dist_left_to_go = lidar_lane_length - poly_length_so_far
        num_of_additional_segs_needed += 1
        all_done = False
    else:  # the closest_seg was long enough
        far_u_val = get_u_value_for_dist(route_segs[closest_seg_num], dist_left_to_go)
        furthest_pt = route_segs[closest_seg_num].get_point(far_u_val)
        all_done = True
    
    while not all_done:
        if closest_seg_num + num_of_additional_segs_needed >= total_seg_num:
            # ran out of segments
            num_of_additional_segs_needed -= 1
            far_u_val = 1.0
            furthest_pt = route_segs[closest_seg_num+num_of_additional_segs_needed].get_point(far_u_val)
            shorty_case = True
            all_done = True
        else:
            poly_length_so_far += route_segs[closest_seg_num+num_of_additional_segs_needed].length
            if poly_length_so_far >= lidar_lane_length:
                far_u_val = get_u_value_for_dist(route_segs[closest_seg_num+num_of_additional_segs_needed], dist_left_to_go)
                furthest_pt = route_segs[closest_seg_num+num_of_additional_segs_needed].get_point(far_u_val)
                all_done = True
            else:
                dist_left_to_go = lidar_lane_length - poly_length_so_far
                num_of_additional_segs_needed += 1
                 
                
    #print('num_of_additional_segs_needed = ', num_of_additional_segs_needed, ' ; left_to_go = ', dist_left_to_go)
    #print('length of now segment = ', route_segments[closest_seg_num+num_of_additional_segs_needed].length)

    num_pts_along_centerline = 10
    poly_pts = np.empty([num_poly_pts,3,2])  # position i,0 will have center point lines ; i,1 will be on the right ; i,2 will be on left
    
    # first center line point is at closest point
    poly_pts[0][0][0] = closest_point[0]
    poly_pts[0][0][1] = closest_point[1]
    # get coords of points to left and right of closest point
    my_u_val = get_u_value_for_dist(route_segs[closest_seg_num], dist_so_far)
    my_heading_rad = route_segs[closest_seg_num].get_heading_rad(my_u_val)
    
    v1 = np.array([math.cos(my_heading_rad + math.pi/2.0), math.sin(my_heading_rad + math.pi/2.0)])
    left_pt = closest_point + (lidar_lane_width/2.0) * v1
    
    poly_pts[0][2][0] = left_pt[0]
    poly_pts[0][2][1] = left_pt[1]
    
    right_pt = closest_point - (lidar_lane_width/2.0) * v1
    
    poly_pts[0][1][0] = right_pt[0]
    poly_pts[0][1][1] = right_pt[1]
    
    if shorty_case:
        # get the total remaining length in the path and set lidar_lane_length to this value
        current_seg_num = closest_seg_num
        dist_remaining = route_segs[closest_seg_num].length - dist_so_far
        total_dist = dist_remaining
        while (current_seg_num+1 < total_seg_num):
            current_seg_num += 1
            total_dist += route_segs[current_seg_num].length
        delta_dist = total_dist/num_pts_along_centerline
        #print('shorty case ; total_dist = ', total_dist, ' ; delta_dist = ', delta_dist)
            
    else:
        delta_dist = lidar_lane_length/num_pts_along_centerline
        
    current_seg_num = closest_seg_num
    dist_on_current_seg = dist_so_far
    how_much_left_on_seg = route_segs[closest_seg_num].length - dist_so_far
   
    how_much_travelled_so_far = 0.0
    how_much_total_left = lidar_lane_length
    
    for i in range(1,num_poly_pts):
        #print('delta = ', delta_dist, ' ; how much left on seg = ', how_much_left_on_seg)
        if how_much_left_on_seg+.05 >= delta_dist:
            dist_on_current_seg += delta_dist
            how_much_left_on_seg -= delta_dist
            how_much_travelled_so_far += delta_dist
            how_much_total_left -= delta_dist
            my_u_val = get_u_value_for_dist(route_segs[current_seg_num], dist_on_current_seg)
            my_center_pt = route_segs[current_seg_num].get_point(my_u_val)

            poly_pts[i][0][0] = my_center_pt[0]
            poly_pts[i][0][1] = my_center_pt[1]
            
            #now add the left and right points
            my_heading_rad = route_segs[current_seg_num].get_heading_rad(my_u_val)
            #print('my_heading_deg = ', my_heading_rad*R2D)
            
            v1 = np.array([math.cos(my_heading_rad + math.pi/2.0), math.sin(my_heading_rad + math.pi/2.0)])
            left_pt = my_center_pt[0:2] + (lidar_lane_width/2.0) * v1
            
            poly_pts[i][2][0] = left_pt[0]
            poly_pts[i][2][1] = left_pt[1]
            
            right_pt = my_center_pt[0:2] - (lidar_lane_width/2.0) * v1
            
            poly_pts[i][1][0] = right_pt[0]
            poly_pts[i][1][1] = right_pt[1]
            
        else:
            current_seg_num += 1

            dist_on_current_seg = delta_dist - how_much_left_on_seg
            how_much_left_on_seg = route_segs[current_seg_num].length + how_much_left_on_seg - delta_dist
            how_much_travelled_so_far += delta_dist
            how_much_total_left -= delta_dist
            my_u_val = get_u_value_for_dist(route_segs[current_seg_num], dist_on_current_seg)
            my_center_pt = route_segs[current_seg_num].get_point(my_u_val)

            poly_pts[i][0][0] = my_center_pt[0]
            poly_pts[i][0][1] = my_center_pt[1]
            
            #now add the left and right points
            my_heading_rad = route_segs[current_seg_num].get_heading_rad(my_u_val)
            #print('my_heading_deg = ', my_heading_rad*R2D)
            
            v1 = np.array([math.cos(my_heading_rad + math.pi/2.0), math.sin(my_heading_rad + math.pi/2.0)])
            left_pt = my_center_pt[0:2] + (lidar_lane_width/2.0) * v1
            
            poly_pts[i][2][0] = left_pt[0]
            poly_pts[i][2][1] = left_pt[1]
            
            right_pt = my_center_pt[0:2] - (lidar_lane_width/2.0) * v1
            
            poly_pts[i][1][0] = right_pt[0]
            poly_pts[i][1][1] = right_pt[1]
                
    
    #print('shorty? = ', shorty_case, ' ; poly_pts = ', poly_pts)
    lidar_poly = np.empty([2*num_poly_pts,2])
    xl = []
    yl = []
    for i in range(num_poly_pts):
        lidar_poly[i][0] = poly_pts[i][1][0]
        lidar_poly[i][1] = poly_pts[i][1][1]

    count_val = num_poly_pts
    for i in range(num_poly_pts-1, -1, -1):
        lidar_poly[count_val][0] = poly_pts[i][2][0]
        lidar_poly[count_val][1] = poly_pts[i][2][1]
        count_val += 1
    
    return lidar_poly
    
#----------------------------------------------------------------------------------