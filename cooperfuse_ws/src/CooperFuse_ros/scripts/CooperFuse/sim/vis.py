# -*- coding: utf-8 -*-
"""
@author: zhaoliang (zhz03@g.ucla.edu)
"""

import matplotlib.pyplot as plt
import numpy as np
import math
from CooperFuse.sim.intersection import  create_intersection
from CooperFuse.mistgen.reeds_shepp_path_planning import plot_arrow

def plot_bbx1(x, y, yaw, l, w, color='b',alpha=0.5):
    # Calculate the four corners of the bounding box
    half_l = l / 2
    half_w = w / 2
    corner1 = [x - half_l, y - half_w]
    corner2 = [x - half_l, y + half_w]
    corner3 = [x + half_l, y + half_w]
    corner4 = [x + half_l, y - half_w]

    # Rotate the bounding box
    rot_matrix = np.array([[math.cos(yaw), -math.sin(yaw)],
                           [math.sin(yaw), math.cos(yaw)]])
    corner1 = np.dot(rot_matrix, corner1)
    corner2 = np.dot(rot_matrix, corner2)
    corner3 = np.dot(rot_matrix, corner3)
    corner4 = np.dot(rot_matrix, corner4)

    # Plot the bounding box
    plt.plot([corner1[0], corner2[0]], [corner1[1], corner2[1]], color, alpha=alpha)
    plt.plot([corner2[0], corner3[0]], [corner2[1], corner3[1]], color, alpha=alpha)
    plt.plot([corner3[0], corner4[0]], [corner3[1], corner4[1]], color, alpha=alpha)
    plt.plot([corner4[0], corner1[0]], [corner4[1], corner1[1]], color, alpha=alpha)

def plot_bbx(x, y, yaw, l, w, color='b', alpha=0.5):
    # Calculate the four corners of the bounding box
    half_l = l / 2
    half_w = w / 2
    corner1 = [-half_l, -half_w]
    corner2 = [-half_l, half_w]
    corner3 = [half_l, half_w]
    corner4 = [half_l, -half_w]

    # Rotate the bounding box
    rot_matrix = np.array([[math.cos(yaw), -math.sin(yaw)],
                           [math.sin(yaw), math.cos(yaw)]])
    corner1 = np.dot(rot_matrix, corner1)
    corner2 = np.dot(rot_matrix, corner2)
    corner3 = np.dot(rot_matrix, corner3)
    corner4 = np.dot(rot_matrix, corner4)

    # Translate the rotated corners to the box center (x, y)
    corner1[0] += x
    corner1[1] += y
    corner2[0] += x
    corner2[1] += y
    corner3[0] += x
    corner3[1] += y
    corner4[0] += x
    corner4[1] += y

    # Plot the bounding box
    plt.plot([corner1[0], corner2[0]], [corner1[1], corner2[1]], color, alpha=alpha)
    plt.plot([corner2[0], corner3[0]], [corner2[1], corner3[1]], color, alpha=alpha)
    plt.plot([corner3[0], corner4[0]], [corner3[1], corner4[1]], color, alpha=alpha)
    plt.plot([corner4[0], corner1[0]], [corner4[1], corner1[1]], color, alpha=alpha)

# The rest of the code remains unchanged.

def bbx_vis_v0(bbxs, color='b', alpha=1.0):
    # plt.figure()
    plt.axis('equal')

    for bbx in bbxs:
        x, y, yaw, l, w, dscore = bbx
        plot_bbx(x, y, yaw, l, w, color, alpha)

    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Bounding Box Visualization')
    plt.grid()
    # plt.show()

def bbx_vis(bbxs, color='b', alpha=1.0):
    # plt.figure()
    plt.axis('equal')

    for bbx in bbxs:
        vid,x, y, yaw, l, w, dscore,ss,kds = bbx
        plot_bbx(x, y, yaw, l, w, color, alpha)

    plt.xlabel('X')
    plt.ylabel('Y')
    # plt.title('Bounding Box Visualization')
    plt.grid()
    
def plot_saved_data_movement(fig_name,bbxs1,bbxs2):
    # define 
    road_width = 50      # 
    lane_width = 3.7     # 
    car_length = 5       # 
    plt.figure(figsize=(10, 10))
    create_intersection()
    bbx_vis(bbxs1, color='k', alpha=0.5)
    bbx_vis(bbxs2, color='k', alpha=1.0)
    for vid,x, y,yaw,l,w,dscore,ss,kds in bbxs1:
        plt.plot(x, y, 'ro')
    plt.xlim(-road_width/2, road_width/2)
    plt.ylim(-road_width/2, road_width/2)
    plt.gca().set_aspect('equal', adjustable='box')
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.title('Intersection with Road Outline and Car Positions')
    # plt.grid(True)
    plt.show()
    plt.savefig(fig_name)
    

def test1():
    bbxs1 = [
        [10, 5, math.pi/14, 2, 1, 0.8],
        [20, 15, math.pi / 4, 3, 1.5, 0.85],
        [30, 25, math.pi / 2, 2.5, 2, 0.9]
    ]
    """
    bbxs2 = [
    [5, 10, math.pi/3, 2, 1, 0.75],
    [15, 20, -math.pi/6, 1.5, 2, 0.85],
    [25, 30, math.pi/4, 2, 1.5, 0.9]
    ]
    """
    bbxs11 = [
    [10, 5, 0, 2, 1, 0.8],
    [20, 15, -math.pi / 6, 3, 1.5, 0.85],
    [30, 25, math.pi / 3, 2.5, 2, 0.9]
    ]
    bbx_vis_v0(bbxs1, color='r', alpha=0.5)
    bbx_vis_v0(bbxs11, color='b', alpha=0.5)    

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

def vis_multi_bbx(fig_title,ori_bbxs,detector1_bbxs,detector2_bbxs,c1,c2,c3,a1,a2,a3):
    """
    This function is to plot three/two bbxes at the same time
    You can also just plot the first bbx

    Parameters
    ----------
    fig_title: str 
        The custom title for the figure 
            and this is also the name for saved file.
    ori_bbxs : TYPE
        DESCRIPTION.
    detector1_bbxs : TYPE
        DESCRIPTION.
    detector2_bbxs : TYPE
        DESCRIPTION.
    c1 : str: 'r'
        color for first bbx.
    c2 :  str: 'r'
        color for first bbx.
    c3 :  str: 'r'
        color for first bbx.
    a1 : TYPE
        DESCRIPTION.
    a2 : TYPE
        DESCRIPTION.
    a3 : TYPE
        DESCRIPTION.

    Returns
    -------
    None.

    """
    
    plt.figure(figsize=(10, 10))
    plt.title(fig_title)
    
    create_intersection()   
    
    # bbx_vis(load_bbx_data, color='k', alpha=0.5)
    # bbx_vis(ss_scores, color='b', alpha=0.5)
    
    bbx_vis(ori_bbxs, color=c1, alpha=a1)
    if detector1_bbxs:
        bbx_vis(detector1_bbxs, color=c2, alpha=a2)
    if detector2_bbxs:
        bbx_vis(detector2_bbxs, color=c3, alpha=a3)
    
    save_name = '/Users/zhaoliang/Documents/Smart_intersection/CooperFuse/results/' + fig_title + '.png'
    plt.savefig(save_name)
        
if __name__ == "__main__":
    yaw1 = np.deg2rad(0)
    yaw2 = np.deg2rad(-10)
    yaw3 = np.deg2rad(10)
    
    bbxs1 = [
        [1,10, 5, yaw1, 2, 1, 0.8,1,1],
        [2,20, 15, yaw2, 2, 1, 0.85,1,1],
        [3,30, 15, yaw3, 2, 1, 0.9,1,1]
    ]
    
    bbx_vis(bbxs1)
    
    plot_arrow(10, 5, yaw1)
    plot_arrow(20, 15, yaw2)
    plot_arrow(30, 15, yaw3)
    
    yaw = calculate_direction_angle(20, 15, 30, 15)
    print(yaw)
    
    


    
