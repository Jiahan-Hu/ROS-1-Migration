import numpy as np
import sys
# from mist import mist_generator 
from mist_new import mist_generator
import time

def main_demo_v010():
    # v 0.1.0 test
    ax = [0.0, 1.0, 1.0,4.0, 5.0,8.0]
    ay = [0.0, 2.0, 4.0,8.0, 2.0,3.0]
    # add a classic case in paper
    # "minimum snap trajectory generation and control for quadrotors"
    ax = [0.0, 5.0,5.0,0.0]
    ay = [0.0, 0.0,6.0,6.0]
    
    waypts_ori = np.array([ax,ay])
    
    T = 10
    v0 = np.array([0,0])
    a0 = np.array([0,0])
    ve = np.array([0,0])
    ae = np.array([0,0])
    
    myMistGen = mist_generator()
    xxs,yys,tts = myMistGen.mist_2d_gen(waypts_ori,v0,a0,ve,ae,T)
    vaj_xy = myMistGen.mist_2d_vaj_gen(xxs,yys,tts)
    myMistGen.mist_2d_vis(waypts_ori,xxs,yys,tts,vaj_xy,True,True,True)

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


    
if __name__ == '__main__':
    
    # v 0.1.0 test
    ax2 = [0.0, 1.0, 1.0,4.0, 5.0,8.0]
    ay2 = [0.0, 2.0, 4.0,8.0, 2.0,3.0]
    # add a classic case in paper
    # "minimum snap trajectory generation and control for quadrotors"
    ax1 = [0.0, 5.0,9.0]
    ay1 = [0.0, 0.0,1.0]
    
    waypts_ori1 = np.array([ax1,ay1])
    waypts_ori2 = np.array([ax2,ay2])
    
    T = 10
    v0 = np.array([0,0])
    a0 = np.array([0,0])
    ve = np.array([0,0])
    ae = np.array([0,0])
    
    myMistGen = mist_generator()
    # xxs,yys,tts,Ex,Ey = myMistGen.mist_2d_gen(waypts_ori1,v0,a0,ve,ae,T)
    start_t = time.time()
    minE1 = wayp_minE(myMistGen,waypts_ori1,v0,a0,ve,ae,T)
    end_t1 =  time.time()
    """
    minE2 = wayp_minE(myMistGen,waypts_ori2,v0,a0,ve,ae,T)
    end_t = time.time()
    t_dur = end_t - start_t
    """
    t_dur1 = end_t1 - start_t
    
    """
    start_t = time.time()
    myMistGen1 = mist_generator()
    minE1 = wayp_minE(myMistGen1,waypts_ori1,v0,a0,ve,ae,T)
    end_t1 =  time.time()

    
    myMistGen2 = mist_generator()
    minE2 = wayp_minE(myMistGen2,waypts_ori2,v0,a0,ve,ae,T)
    end_t = time.time()
    t_dur = end_t - start_t
    t_dur1 = end_t1 - start_t
    """
    
    # t_dur1 = 0.00738s t_dur = 0.017s
    
    
    
    
   