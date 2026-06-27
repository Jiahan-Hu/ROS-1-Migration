
#import sys
#sys.path.append("..")
# from mist import mist_generator
import time
import math 
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

from CooperFuse.mistgen.mist_new import mist_generator
from CooperFuse.mistgen.reeds_shepp_path_planning import reeds_shepp_path_planning,remove_duplicates,vis_wayp,remove_same_neighbor,plot_arrow
from CooperFuse.mistgen.turning_points import check_turning_points,find_turning_points
# from sim.vis import bbx_vis
        

def wayp_minE(myMistGen,waypts_ori,v0,a0,ve,ae,T):
    xxs,yys,tts,Ex,Ey = myMistGen.mist_2d_gen(waypts_ori,v0,a0,ve,ae,T)
    show_xy = False
    if show_xy:
        vaj_xy = myMistGen.mist_2d_vaj_gen(xxs,yys,tts)
        myMistGen.mist_2d_vis(waypts_ori,xxs,yys,tts,vaj_xy,True,True,True)
    # minE = cal_minE(myMistGen)
    return Ex + Ey  

def wayp_minE_debug(myMistGen,waypts_ori,v0,a0,ve,ae,T):
    xxs,yys,tts,Ex,Ey = myMistGen.mist_2d_gen(waypts_ori,v0,a0,ve,ae,T)
    show_xy = False
    if show_xy:
        vaj_xy = myMistGen.mist_2d_vaj_gen(xxs,yys,tts)
        myMistGen.mist_2d_vis(waypts_ori,xxs,yys,tts,vaj_xy,True,True,True)
    # minE = cal_minE(myMistGen)
    return Ex + Ey,xxs,yys

def cal_minE(myMistGen):
    Qx = myMistGen.Q_x 
    px = myMistGen.px
    Ex = np.dot(np.transpose(px), np.dot(Qx, px))
    Qy = myMistGen.Q_y 
    py = myMistGen.py
    Ey = np.dot(np.transpose(py), np.dot(Qy, py))
    minE = Ex + Ey
    return minE     

def select_points_with_equal_intervals(xs, num_points):
    if num_points < 3:
        raise ValueError("num_points should be at least 3.")
        
    if len(xs) == num_points or len(xs) < num_points:
        return xs
    else:

        interval = (len(xs) - 1) / (num_points - 1)
        selected_points = [xs[0]]  # Start point
    
        for i in range(1, num_points - 1):
            index = round(i * interval)
            selected_points.append(xs[index])
    
        selected_points.append(xs[-1])  # End point
        return selected_points

def minE_calculation(bbx_pre,bbx_curr):
    vid_p,start_x,start_y,start_yaw,_,_,_,_,_ = bbx_pre
    vid,end_x,end_y,end_yaw,_,_,_,_,_ = bbx_curr
    curvature = 1
    step_size = 0.3 # 0.05
    xs, ys, yaws, modes, lengths = reeds_shepp_path_planning(start_x, start_y,
                                                             start_yaw, end_x,
                                                             end_y, end_yaw,
                                                             curvature,
                                                             step_size)    
    
    num_points_to_select = 4  # Change this to select a different number of points
    s_xs = select_points_with_equal_intervals(xs, num_points_to_select)
    s_ys = select_points_with_equal_intervals(ys, num_points_to_select)
    
    T = 1
    v0 = np.array([0,0])
    a0 = np.array([0,0])
    ve = np.array([0,0])
    ae = np.array([0,0])  
    waypts_ori = np.array([s_xs,s_ys])
    myMistGen = mist_generator()
    minE = wayp_minE(myMistGen,waypts_ori,v0,a0,ve,ae,T)    
    
    # minE = 0
    
    return minE[0][0]

def minE_calculation_v2(bbx_pre,bbx_curr):
    vid_p,start_x,start_y,start_yaw,_,_,_,_,_ = bbx_pre
    vid,end_x,end_y,end_yaw,_,_,_,_,_ = bbx_curr
    curvature = 1
    step_size = 0.3 # 0.05
    xs, ys, yaws, modes, lengths = reeds_shepp_path_planning(start_x, start_y,
                                                             start_yaw, end_x,
                                                             end_y, end_yaw,
                                                             curvature,
                                                             step_size) 
    uxs,uys = remove_same_neighbor(xs,ys)
    # check if it's at the same location
    # turning_points = find_turning_points(uxs,uys)
    check = check_turning_points(uxs,uys)  # if true, then same position
    if check:
        coefficients = load_saved_data()
        yaw_diff_deg = yaw_diff_convert(start_yaw,end_yaw)
        minE1 = polynomial_fit(yaw_diff_deg,coefficients)
        print("same position")
    else:
        
        num_points_to_select = 4  # Change this to select a different number of points
        s_xs = select_points_with_equal_intervals(uxs, num_points_to_select)
        s_ys = select_points_with_equal_intervals(uys, num_points_to_select)
        waypts_ori = np.array([s_xs,s_ys])
        v0 = np.array([0,0])
        a0 = np.array([0,0])
        ve = np.array([0,0])
        ae = np.array([0,0])  
        myMistGen = mist_generator()
        minE1,xxs,yys = wayp_minE_debug(myMistGen,waypts_ori,v0,a0,ve,ae,T=0.1)
        
    return minE1[0][0]

def minE_calculation_v3(bbx_pre,bbx_curr):
    vid_p,start_x,start_y,start_yaw,_,_,_,_,_ = bbx_pre
    vid,end_x,end_y,end_yaw,_,_,_,_,_ = bbx_curr
    curvature = 1
    step_size = 0.3 # 0.05
    xs, ys, yaws, modes, lengths = reeds_shepp_path_planning(start_x, start_y,
                                                             start_yaw, end_x,
                                                             end_y, end_yaw,
                                                             curvature,
                                                             step_size) 
    uxs,uys = remove_same_neighbor(xs,ys)
    # check if it's at the same location
    # turning_points = find_turning_points(uxs,uys)
    check = check_turning_points(uxs,uys)  # if true, then same position
    if check:
        coefficients = load_saved_data()
        yaw_diff_deg = yaw_diff_convert(round(start_yaw,2),round(end_yaw,2))
        # print(yaw_diff_deg)
        minE1 = polynomial_fit(round(yaw_diff_deg,2),coefficients)
        print("same position:",minE1)
    else:
        dt = 1
        num_points_to_select = 4  # Change this to select a different number of points
        s_xs = select_points_with_equal_intervals(uxs, num_points_to_select)
        s_ys = select_points_with_equal_intervals(uys, num_points_to_select)
        waypts_ori = np.array([s_xs,s_ys])
        
        ave_v = calculate_velocity(start_x, start_y, end_x, end_y, dt)
        # yaw = calculate_direction_angle(start_x, start_y, end_x, end_y)
        yaw_d1 = calculate_direction_angle(xs[0], ys[0], xs[1], ys[1])
        yaw_d2 = calculate_direction_angle(xs[-2], ys[-2], xs[-1], ys[-1])
        
        v0 = calculate_velocity_vec(ave_v,start_yaw,yaw_d1)
        a0 = np.array([0,0])
        ve = calculate_velocity_vec(ave_v,end_yaw,yaw_d2)
        ae = np.array([0,0])
        
        myMistGen = mist_generator()
        minE1,xxs,yys = wayp_minE_debug(myMistGen,waypts_ori,v0,a0,ve,ae,T=dt)
        minE1 = minE1[0][0]
        
    return minE1

def calculate_velocity(start_x, start_y, end_x, end_y, dt):
    # Calculate distance using Euclidean distance formula
    distance = ((end_x - start_x)**2 + (end_y - start_y)**2)**0.5
    
    # Calculate velocity
    velocity = distance / dt
    
    return velocity

def calculate_velocity_vec(velocity, start_yaw,ref_yaw):
    """
    start_yaw: in radians
    ref_yaw: this is the reference yaw angle 
        in radians 
    """
    
    # Convert start_yaw to radians
    # start_yaw_rad = math.radians(start_yaw)
    
    # get the correct direction
    # print("oir:",start_yaw)
    if start_yaw * ref_yaw > 0:
        dir_flg = 1.0
        start_yaw = start_yaw * dir_flg
    else:
        dir_flg = -1.0 
        start_yaw =  - (np.deg2rad(180) - start_yaw)
        # start_yaw = start_yaw * dir_flg
    # print(start_yaw)
    # Calculate initial velocity components
    vx = velocity * math.cos(start_yaw) 
    vy = velocity * math.sin(start_yaw) 
    
    return np.array([vx, vy])

        
def yaw_diff_convert(start_yaw,end_yaw):
    """
    start_yaw: in radian 
    end_yaw: in radian
    """
    yaw_diff = abs(end_yaw - start_yaw)
    yaw_diff_deg = np.rad2deg(yaw_diff)
    if yaw_diff_deg > 90:
        yaw_diff_deg = 180 - yaw_diff_deg
        
    return yaw_diff_deg         
    
    
def output_key_var(start_x,start_y,start_yaw,end_x,end_y,deg,T = 1):

    curvature = 1
    step_size = 0.3 # 0.05    
    
    end_yaw = np.deg2rad(deg)

    xs, ys, yaws, modes, lengths = reeds_shepp_path_planning(start_x, start_y,
                                                             start_yaw, end_x,
                                                             end_y, end_yaw,
                                                             curvature,
                                                             step_size)
    uxs,uys = remove_same_neighbor(xs,ys)
    
    turning_points = find_turning_points(uxs,uys)
    
    check = check_turning_points(uxs,uys)  # if true, then same position
    
    num_points_to_select = 4  # Change this to select a different number of points
    
    s_xs = select_points_with_equal_intervals(uxs, num_points_to_select)
    s_ys = select_points_with_equal_intervals(uys, num_points_to_select)
    waypts_ori = np.array([s_xs,s_ys])
    
    v0 = np.array([0,0])
    a0 = np.array([0,0])
    ve = np.array([0,0])
    ae = np.array([0,0])  
    myMistGen = mist_generator()
    minE1,xxs,yys = wayp_minE_debug(myMistGen,waypts_ori,v0,a0,ve,ae,T)
    
    # vis_wayp(uxs,uys)
    # vis_wayp(xxs,yys)
    return check, deg,minE1

def polynomial_fit_param(degs,minEs,vis=False,if_save=True):
    order = 3
    coefficients = np.polyfit(degs, minEs, order)
    poly_fit = np.poly1d(coefficients)
    x_fit = np.linspace(min(degs), max(degs), 100)
    y_fit = poly_fit(x_fit)
    
    # plot it
    if vis == True:
        plt.scatter(degs, minEs, label='original')
        plt.plot(x_fit, y_fit, color='red', label='polynomial_fit')
    
    if if_save==True:
        npy_name = "/Users/zhaoliang/Documents/Smart_intersection/CooperFuse/mistgen/ploy_param/poly_p_T1.npy"
        np.save(npy_name, coefficients)
        
    return coefficients

def polynomial_fit(deg,coefficients):
    
    poly_fit = np.poly1d(coefficients)
    y_fit = poly_fit(deg)
    if y_fit < 0.0:
        y_fit = 0.0 
    
    return y_fit

def load_saved_data():

    # Specify the path to the .npy file
    npy_name = "/home/zhaoliang/zzl/zhz03_github/Cooperfuse_framework/Cooperfuse_Ws/src/CooperFuse_ros/scripts/CooperFuse/mistgen/ploy_param/poly_p_T1.npy"
    
    # Load the data from the .npy file
    loaded_data = np.load(npy_name) 
    
    return loaded_data   

def calculate_direction_angle(start_x, start_y, end_x, end_y):
    """
    

    Parameters
    ----------
    start_x : TYPE
        DESCRIPTION.
    start_y : TYPE
        DESCRIPTION.
    end_x : TYPE
        DESCRIPTION.
    end_y : TYPE
        DESCRIPTION.

    Returns
    -------
    direction_angle 
        in radians 

    """
    
    dx = end_x - start_x
    dy = end_y - start_y
    
    direction_angle = math.degrees(math.atan2(dy, dx))
 
    return np.deg2rad(direction_angle)

# def angle_flip(yaw):
    
    

def test1():
    # Test data
    bbx_pre =  [1.        , -3.03983043, -4.70374221,  1.67283675,  5.16      ,1.89      ,  0.        ,  0.        ,  0.        ]
    bbx_curr = [1.0,
     -2.8570322345466703,
     -5.959608297282266,
     1.7153367522675207,
     5.16,
     1.89,
     0.0,
     [1.0, 1.0],
     0.0]
    # minE = minE_calculation_v3(bbx_pre,bbx_curr)
    
    vid_p,start_x,start_y,start_yaw,_,_,_,_,_ = bbx_pre
    vid,end_x,end_y,end_yaw,_,_,_,_,_ = bbx_curr
    curvature = 1
    step_size = 0.3 # 0.05
    xs, ys, yaws, modes, lengths = reeds_shepp_path_planning(start_x, start_y,
                                                             start_yaw, end_x,
                                                             end_y, end_yaw,
                                                             curvature,
                                                             step_size) 
    uxs,uys = remove_same_neighbor(xs,ys)
    # check if it's at the same location
    # turning_points = find_turning_points(uxs,uys)
    check = check_turning_points(uxs,uys)  # if true, then same position
    if check:
        coefficients = load_saved_data()
        yaw_diff_deg = yaw_diff_convert(round(start_yaw,2),round(end_yaw,2))
        print(yaw_diff_deg)
        minE1 = polynomial_fit(round(yaw_diff_deg,2),coefficients)
        print("same position:",minE1)
    else:
        dt = 0.1 
        num_points_to_select = 4  # Change this to select a different number of points
        s_xs = select_points_with_equal_intervals(uxs, num_points_to_select)
        s_ys = select_points_with_equal_intervals(uys, num_points_to_select)
        waypts_ori = np.array([s_xs,s_ys])
        
        ave_v = calculate_velocity(start_x, start_y, end_x, end_y, dt)
        flag = calculate_direction_angle(start_x, start_y, end_x, end_y)
        
        v0 = calculate_velocity_vec(ave_v,start_yaw) * flag
        a0 = np.array([0,0])
        ve = calculate_velocity_vec(ave_v,end_yaw) * flag
        ae = np.array([0,0])  
        myMistGen = mist_generator()
        minE1,xxs,yys = wayp_minE_debug(myMistGen,waypts_ori,v0,a0,ve,ae,T=dt)
        minE1 = minE1[0][0]
        
    plt.figure()
    # plt.scatter(xs, ys,color='b')
    # plt.scatter(s_xs,s_ys,color='y')
    plt.plot(xxs,yys,color='red')
    
if __name__ == '__main__':
    """
    bbx_pre =  [ 9.        ,  4.36636606, -7.15879779, -1.39791944,  4.97      ,
            1.87      ,  0.        ,  0.        ,  0.        ]
    bbx_curr = [9.0,
     4.102268463933663,
     -5.781693114729622,
     -1.3813194375626292,
     4.97,
     1.87,
     0.0,
     [1.0, 1.0],
     'E17988211236.509235']
    
    bbx_pre = [ 5.00000000e+00,  9.58673153e+00, -4.34606188e+00,  6.12468793e-03,
            5.28000000e+00,  1.86000000e+00,  0.00000000e+00,  0.00000000e+00,
            0.00000000e+00]
    bbx_curr = [5.0,
     11.098936124460911,
     -4.3703731360241855,
     -0.0160753120666094,
     5.28,
     1.86,
     0.0,
     [1.0, 1.0],
     'E45200.999073680774']
    """
    """
    bbx_pre = [7.        , 4.63149675, 1.57740686, 2.10067817, 5.07      ,
           1.83      , 0.        , 0.        , 0.        ]
    bbx_curr = [7.0,
     5.335987304396341,
     0.44236418981657244,
     2.126278173108432,
     5.07,
     1.83,
     0.0,
     [1.0, 1.0],
     'E121963.05151055568']
    
    bbx_pre = [  3.        , -19.89059228,   4.23133215,   0.20677594,
             4.89      ,   1.84      ,   0.        ,   0.        ,
             0.        ]
    bbx_curr = [3.0,
     -18.441345604109454,
     4.607220294101754,
     0.2537759425778175,
     4.89,
     1.84,
     0.0,
     [1.0, 1.0],
     'E17.43440188015992']
    
    bbx_two = []
    bbx_two.append(bbx_pre)
    bbx_two.append(bbx_curr)
    
    # minE = minE_calculation_v3(bbx_pre,bbx_curr)
    
    vid_p,start_x,start_y,start_yaw,_,_,_,_,_ = bbx_pre
    vid,end_x,end_y,end_yaw,_,_,_,_,_ = bbx_curr
    curvature = 1
    step_size = 0.3 # 0.05
    xs, ys, yaws, modes, lengths = reeds_shepp_path_planning(start_x, start_y,
                                                             start_yaw, end_x,
                                                             end_y, end_yaw,
                                                             curvature,
                                                             step_size) 
    uxs,uys = remove_same_neighbor(xs,ys)

    dt = 1
    
    
    # start_x, start_y, end_x, end_y = 0,0,5,5
    ave_v = calculate_velocity(start_x, start_y, end_x, end_y, dt)

    yaw_d1 = calculate_direction_angle(xs[0], ys[0], xs[1], ys[1])
    yaw_d2 = calculate_direction_angle(xs[-2], ys[-2], xs[-1], ys[-1])
    
    # plot_arrow(end_x, end_y, start_yaw)
    # plt.plot(start_x, start_y,color="b",marker='o')
    # plt.plot(end_x, end_y,color="r",marker='o')
    
    v0 = calculate_velocity_vec(ave_v,start_yaw,yaw_d1) 
    a0 = np.array([0,0])
    ve = calculate_velocity_vec(ave_v,end_yaw,yaw_d2) 
    ae = np.array([0,0])
      
    uxs,uys = remove_same_neighbor(xs,ys)
    # check if it's at the same location
    # turning_points = find_turning_points(uxs,uys)
    check = check_turning_points(uxs,uys)  # if true, then same position
    
    num_points_to_select = 4  # Change this to select a different number of points
    s_xs = select_points_with_equal_intervals(uxs, num_points_to_select)
    s_ys = select_points_with_equal_intervals(uys, num_points_to_select)
    waypts_ori = np.array([s_xs,s_ys])
    
    myMistGen = mist_generator()
    minE1,xxs,yys = wayp_minE_debug(myMistGen,waypts_ori,v0,a0,ve,ae,T=dt)
    print(minE1)

    plt.figure()
    plt.scatter(xs, ys,color='red')
    plt.scatter(xs[0],ys[0],color='blue')  
    # plt.scatter(s_xs,s_ys,color="g")
    # plt.scatter(xxs,yys,color='k')
    plt.plot(xxs, yys, c='k')
    
    minE2 = minE_calculation_v3(bbx_pre,bbx_curr)
    bbx_vis(bbx_two)
    """
    

    



    
    
    
    
    
    
    
    
