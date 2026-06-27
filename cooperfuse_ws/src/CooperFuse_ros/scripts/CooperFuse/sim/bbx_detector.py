# -*- coding: utf-8 -*-
"""
@author: zhaoliang (zhz03@g.ucla.edu)
"""

import random
import math

class Detector:
    def __init__(self, x_noise, y_noise, yaw_noise, l_noise, w_noise, dscore_min, dscore_max,sense_k):
        self.x_noise = x_noise
        self.y_noise = y_noise
        self.yaw_noise = yaw_noise
        self.l_noise = l_noise
        self.w_noise = w_noise
        self.dscore_min = dscore_min
        self.dscore_max = dscore_max
        self.ego_id = random.randint(1000, 9999)
        self.randsense_num = sense_k
        self.randsense_num_list  = []

    def generate_bbx_v0(self, xs, ys, yaws, ls, ws):
        bbxs = []
        ego_id = random.randint(1000, 9999)
        vid = 0
        for x, y, yaw, l, w in zip(xs, ys, yaws, ls, ws):
            vid += 1
            fvid = ego_id + vid
            x_noisy = x + random.normalvariate(0, self.x_noise)
            y_noisy = y + random.normalvariate(0, self.y_noise)
            yaw_noisy = yaw + random.normalvariate(0, self.yaw_noise)
            l_noisy = l + random.normalvariate(0, self.l_noise)
            w_noisy = w + random.normalvariate(0, self.w_noise)
            dscore = random.uniform(self.dscore_min, self.dscore_max)
            ss = 0
            kds = 0

            bbx = [fvid,x_noisy, y_noisy, yaw_noisy, l_noisy, w_noisy, dscore,ss,kds]
            bbxs.append(bbx)

        return bbxs
    def generate_bbx(self, gt_bbxs):
        bbxs = []
        
        vid = 0
        for vid,x, y, yaw, l, w, dscore,ss,kds in gt_bbxs:
            vid += 1
            fvid = self.ego_id + vid
            x_noisy = x + random.normalvariate(0, self.x_noise)
            y_noisy = y + random.normalvariate(0, self.y_noise)
            yaw_noisy = yaw + random.normalvariate(0, self.yaw_noise)
            l_noisy = l + random.normalvariate(0, self.l_noise)
            w_noisy = w + random.normalvariate(0, self.w_noise)
            dscore = random.uniform(self.dscore_min, self.dscore_max)
            ss = 0
            kds = 0

            bbx = [fvid,x_noisy, y_noisy, yaw_noisy, l_noisy, w_noisy, dscore,ss,kds]
            bbxs.append(bbx)

        return bbxs
    
    def random_sense(self,gt_bbxs):
        """
        This function is to randomly determine sensing bbx from the gt_bbxs

        Parameters
        ----------
        gt_bbxs : TYPE
            DESCRIPTION.

        Raises
        ------
        ValueError
            DESCRIPTION.

        Returns
        -------
        random_sense_bbxs : TYPE
            DESCRIPTION.

        """
        
        n = len(gt_bbxs)
        if self.randsense_num > n:
            raise ValueError("gt_bbx number should be less than or equal to randsense_num")
            
        numbers = list(range(n))  # create list from 0 to n-1 
        self.randsense_num_list = random.sample(numbers, self.randsense_num)  # select self.randsense_num number from list
        
        random_sense_bbxs = []
        for i in self.randsense_num_list:
            random_sense_bbxs.append(gt_bbxs[i])
        
        return random_sense_bbxs
    
    def select_sense(self,gt_bbxs):
        if len(self.randsense_num_list) == 0:
            raise ValueError("self.randsense_num_list is empty! Need to run random_sense first! ")
        random_sense_bbxs = []
        for i in self.randsense_num_list:
            random_sense_bbxs.append(gt_bbxs[i])
        
        return random_sense_bbxs
        
            
if __name__ == "__main__":

    detector = Detector(x_noise=0.001, y_noise=0.001, yaw_noise=0.05, l_noise=0.8, w_noise=0.6, dscore_min=0.85, dscore_max=0.9)
    xs = [10, 20, 30]
    ys = [5, 15, 25]
    yaws = [0, math.pi / 4, math.pi / 2]  # Yaw angles in radians
    ls = [2, 3, 2.5]
    ws = [1, 1.5, 2]
    
    # bbxs = detector.generate_bbx_v0(xs, ys, yaws, ls, ws)
    # print(bbxs)


