#! /usr/bin/env python
# coding: utf-8

import numpy as np
from marker_decoder import decode_marker_all
from CooperFuse.mistgen.mist_new import mist_generator
from CooperFuse.mistgen.reeds_shepp_path_planning import reeds_shepp_path_planning,remove_duplicates,vis_wayp,remove_same_neighbor,plot_arrow
from CooperFuse.mistgen.turning_points import check_turning_points,find_turning_points
from CooperFuse.mistgen.comb_planning_minE import wayp_minE_debug,load_saved_data,yaw_diff_convert,polynomial_fit,select_points_with_equal_intervals,calculate_direction_angle,calculate_velocity_vec

def minE_calculation_3d(marker_pre,marker_cur):
    """
    This is function to calculate the mininum Energy based on previous marker and current marker

    Parameters
    ----------
    marker_pre : visualization_msgs/marker
        DESCRIPTION.
    marker_cur : visualization_msgs/marker
        DESCRIPTION.

    Returns
    -------
    minE : float
        Minimum energy cost from one state to another state

    """
    cur_x, cur_y, cur_z, cur_yaw, cur_l, cur_w, cur_h, cur_velo, cur_ID, cur_ds, cur_ss, cur_kds = decode_marker_all(marker_cur)
    pre_x, pre_y, pre_z, pre_yaw, pre_l, pre_w, pre_h, pre_velo, pre_ID, pre_ds, pre_ss, pre_kds = decode_marker_all(marker_pre)
    
    curvature = 1
    step_size = 0.3 # 0.05
    
    start_x = pre_x
    start_y = pre_y
    start_yaw = pre_yaw
    end_x = cur_x
    end_y = cur_y
    end_yaw = cur_yaw
    
    xs, ys, yaws, modes, lengths = reeds_shepp_path_planning(start_x, start_y,
                                                         start_yaw, end_x,
                                                         end_y, end_yaw,
                                                         curvature,
                                                         step_size)
    
    uxs,uys = remove_same_neighbor(xs,ys)
    check = check_turning_points(uxs,uys)  # if true, then same position
    
    if check:
        coefficients = load_saved_data()
        yaw_diff_deg = yaw_diff_convert(round(start_yaw,2),round(end_yaw,2))
        # print(yaw_diff_deg)
        minE = polynomial_fit(round(yaw_diff_deg,2),coefficients)
        # print("same position:",minE)
    else:
        dt = 1
        num_points_to_select = 4  # Change this to select a different number of points
        s_xs = select_points_with_equal_intervals(uxs, num_points_to_select)
        s_ys = select_points_with_equal_intervals(uys, num_points_to_select)
        waypts_ori = np.array([s_xs,s_ys])
        
        start_v = pre_velo
        end_v = cur_velo
        # ave_v = calculate_velocity(start_x, start_y, end_x, end_y, dt)
        
        # yaw = calculate_direction_angle(start_x, start_y, end_x, end_y)
        yaw_d1 = calculate_direction_angle(xs[0], ys[0], xs[1], ys[1])
        yaw_d2 = calculate_direction_angle(xs[-2], ys[-2], xs[-1], ys[-1])
        
        v0 = calculate_velocity_vec(start_v,start_yaw,yaw_d1)
        a0 = np.array([0,0])
        ve = calculate_velocity_vec(end_v,end_yaw,yaw_d2)
        ae = np.array([0,0])
        
        myMistGen = mist_generator()
        minE1,xxs,yys = wayp_minE_debug(myMistGen,waypts_ori,v0,a0,ve,ae,T=dt)
        minE = minE1[0][0]
    
    return minE

if __name__ == '__main__':
    pass 