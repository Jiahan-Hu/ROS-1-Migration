# -*- coding: utf-8 -*-
"""
@author: zhaoliang (zhz03@g.ucla.edu)
"""
import matplotlib.pyplot as plt
import numpy as np
import math
from marker_decoder import decode_marker_all
from CooperFuse.sim.vis import plot_bbx

def bbx_marker_vis(bbx_markers,color='b',alpha=1.0,if_show=False):
    # plt.figure()
    plt.axis('equal')

    for marker in bbx_markers.markers:
        m1_x, m1_y, m1_z, m1_yaw, m1_l, m1_w, m1_h, _, m1_ID, _, _, _ = decode_marker_all(marker)
        plot_bbx(m1_x, m1_y, m1_yaw, m1_l, m1_w, color, alpha)

    plt.xlabel('X')
    plt.ylabel('Y')
    # plt.title('Bounding Box Visualization')
    plt.grid()
    if if_show:
        plt.show()

