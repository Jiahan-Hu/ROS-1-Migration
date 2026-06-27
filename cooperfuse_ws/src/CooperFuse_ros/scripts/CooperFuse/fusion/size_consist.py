
def size_consistent_score(bbxs_pre,bbxs_curr):
    ss_bbxs = []
    for vid,x,y,yaw,l,w,dscore,ss,kds in bbxs_curr:
        for vid_p,x_p,y_p,yaw_p,l_p,w_p,dscore_p,ss_p,kds_p in bbxs_pre:
            if vid == vid_p:
                ss_l = min(l,l_p)/max(l,l_p)
                ss_w = min(w,w_p)/max(w,w_p)
                ss = [ss_l,ss_w]
                break 
            else:
                ss = [dscore,dscore]
        ss_bbx = [vid,x,y,yaw,l,w,dscore,ss,kds]
        ss_bbxs.append(ss_bbx)
            
    return ss_bbxs
    
if __name__ == "__main__":
    # bbxs_pre
    # bbxs_current
    # from detection
    
    # input: bbxs = [id,x,y,yaw,l,w,dscore]
    # output: bbxs = [id,x,y,yaw,l,w,dscore,kds,ss]
    
    # Test data
    bbxs_pre = [
        (1, 0.0, 0.0, 0.0, 5.0, 2.2, 0.9),
        (2, -1.0, 1.0, 45.0, 4.0, 1.5, 0.8),
        (3, 2.0, -2.0, -30.0, 6.0, 1.8, 0.7),
        (4, 2.0, -2.0, -30.0, 6.0, 1.4, 0.7)
    ]
    
    bbxs_curr = [
        (1, 0.0, 0.0, 0.0, 5.2, 1.8, 0.9),
        (5, -1.0, 1.0, 45.0, 3.8, 1.6, 0.85),
        (3, 2.0, -2.0, -30.0, 5.8, 1.9, 0.75),
        (6, -1.0, 1.0, 45.0, 3.8, 1.6, 0.85),
        
    ]
    
    ss_scores = size_consistent_score(bbxs_pre, bbxs_curr)
    for ss_bbx in ss_scores:
        print(ss_bbx)
    