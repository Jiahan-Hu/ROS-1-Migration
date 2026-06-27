# -*- coding: utf-8 -*-
"""
@author: zhaoliang (zhz03@g.ucla.edu)
"""

from bbx_detector import Detector
from vis import bbx_vis
from motion_model import ctrv_motion_model
import math
from motion_model import generate_rand_v0s,generate_rand_omegas,generate_rand_accels,generate_rand_phis

def test_1():
    detector = Detector(x_noise=0.1, y_noise=0.2, yaw_noise=0.5, l_noise=0.1, w_noise=0.1, dscore_min=0.85, dscore_max=0.95)
    xs = [10, 20, 30]
    ys = [5, 15, 25]
    yaws = [0, math.pi / 4, math.pi / 2]  # Yaw angles in radians
    ls = [2, 3, 2.5]
    ws = [1, 1.5, 2]

    bbxs1 = detector.generate_bbx(xs, ys, yaws, ls, ws)
    bbxs2 = detector.generate_bbx(xs, ys, yaws, ls, ws)
    bbx_vis(bbxs1, color='r', alpha=0.5)
    bbx_vis(bbxs2, color='b', alpha=0.5) 
        
if __name__ == "__main__":   

    bbx1 = [0.0, 0.0, 0.0, 4.0, 2.0, 0.8]
    bbx2 = [10.0, 20.0, math.pi/4, 3.0, 1.5, 0.7]
    bbx3 = [-13.0, -10.0, -math.pi/2, 5.0, 3.0, 0.9]
    bbxs = [bbx1, bbx2, bbx3]
    
    """
    v0s = [10.0, 0.0, 9]
    omegas = [0.1, 0.0, -0.3]
    accels = [1.0, 0.0, 1.2]
    phis = [0.0, 0.0, 0.0]
    """
    
    v0s = generate_rand_v0s(3)
    omegas = generate_rand_omegas(v0s)
    accels = generate_rand_accels(omegas)
    phis = generate_rand_phis(accels)

    dt = 0.1

    # Move the vehicles one step into the future
    new_bbxs = ctrv_motion_model(bbxs, v0s, omegas, accels, phis, dt)
    
    bbx_vis(bbxs, color='b', alpha=0.5)
    bbx_vis(new_bbxs, color='b', alpha=1.0)

    

