# -*- coding: utf-8 -*-
"""
@author: zhaoliang (zhz03@g.ucla.edu)
"""

import math
import numpy as np
import random

def ctrv_motion_model_single(x, y, yaw, v, omega, a, fyi, dt):
    new_x = x + (v / omega) * (math.sin(yaw + omega * dt) - math.sin(yaw))
    new_y = y + (v / omega) * (math.cos(yaw) - math.cos(yaw + omega * dt))
    new_yaw = yaw + omega * dt
    new_v = v + a * dt
    new_omega = omega + fyi * dt
    
    return new_x, new_y, new_yaw, new_v, new_omega

def ctrv_motion_model1(bbxs, v0s, omegas, accels, fyis, dt):
    new_positions = []
    new_velocities = []
    new_omegas = []

    for i, (bbx, v0, omega, a, fyi) in enumerate(zip(bbxs, v0s, omegas, accels, fyis)):
        x, y, yaw, l, w, dscore = bbx
        new_x, new_y, new_yaw = x, y, yaw  # Initialize new positions with the current positions

        # Compute new positions
        if abs(omega) > 1e-6:  # Avoid division by zero
            new_x = x + (v0 / omega) * (math.sin(yaw + omega * dt) - math.sin(yaw))
            new_y = y + (v0 / omega) * (math.cos(yaw) - math.cos(yaw + omega * dt))
            new_yaw = yaw + omega * dt

        # Compute new velocities and omegas
        new_v = v0 + a * dt
        new_omega = omega + fyi * dt

        new_positions.append([new_x, new_y, new_yaw, l, w, dscore])
        new_velocities.append(new_v)
        new_omegas.append(new_omega)

    return new_positions, new_velocities, new_omegas

def ctrv_motion_model(bbxs, v0s, omegas, accels, phis, dt):
    new_bbxs = []

    for i, (bbx, v0, omega, a, fyi) in enumerate(zip(bbxs, v0s, omegas, accels, phis)):
        vid, x, y, yaw, l, w, dscore,ss,kds = bbx
        new_x, new_y, new_yaw = x, y, yaw  # Initialize new positions with the current positions

        # Compute new positions
        """
        if abs(omega) > 1e-6:  # Avoid division by zero
            new_yaw = yaw + omega * dt
            new_x = x + (v0 / omega) * (math.sin(yaw + omega * dt) - math.sin(yaw))
            new_y = y + (v0 / omega) * (math.cos(yaw) - math.cos(yaw + omega * dt))
        """
        omega += fyi * dt
        new_yaw = yaw + omega * dt
        v = v0 + a * dt
        new_x = x + v * np.cos(new_yaw) * dt
        new_y = y + v * np.sin(new_yaw) * dt  

        new_bbxs.append([vid,new_x, new_y, new_yaw, l, w, dscore,ss,kds])  # Keep the previous dscore

    return new_bbxs

def single_example():
    x0, y0, yaw0, v0, omega0, a0, fyi0 = 0.0, 0.0, 0.0,10.0, 0.1, 1.0, 0.0
    
    
    dt = 0.1
    
    # Move the vehicle one step into the future
    x1, y1, yaw1, v1, omega1 = ctrv_motion_model(x0, y0, yaw0, v0, omega0, a0, fyi0, dt)
    
    print(f"Current position: ({x0}, {y0}), Yaw angle: {yaw0}")
    print(f"Next position: ({x1}, {y1}), Next yaw angle: {yaw1}")
    print(f"Velocity: {v1}, Turning velocity: {omega1}")    

def generate_rand_v0s(car_num,random_factor=2):
    
    v0s = []
    if random_factor == 2:
        
        while len(v0s) < car_num:
            rand_var = random.random()
            random_dir = random.choice([-1, 1])
            if rand_var < 0.1:
                vs= round(random.uniform(12, 17), 2) * random_dir
            elif rand_var >= 0.1 and rand_var < 0.6:
                vs= round(random.uniform(4, 12), 2) * random_dir
            else:
                vs = 0.0
            v0s.append(vs)
    elif random_factor == 1:
        while len(v0s) < car_num:
            random_dir = random.choice([-1, 1])
            vs= round(random.uniform(12, 17), 2) * random_dir
            v0s.append(vs)
    else: # radom_factor = 0
        v0s = [0.0] * car_num
    return v0s

def generate_rand_omegas(v0s):
    omegas= []
    for vs in v0s:
        if vs == 0.0:
            omega = 0.0
        else:    
            rand_var = random.random()
            if rand_var < 0.3:
                omega = round(random.uniform(-0.5, -0.2), 2)
            if rand_var >=0.3 and rand_var < 0.7:
                omega = round(random.uniform(-0.2, 0.2), 2)
            else:
                omega = round(random.uniform(0.2, 0.5), 2)
        omegas.append(omega)
    return omegas        

def generate_rand_accels(omegas):
    accels= []
    for omega in omegas:
        if omega == 0.0:
            accel = 0.0
        else:    
            rand_var = random.random()
            if rand_var < 0.3:
                accel = round(random.uniform(-1.5, -0.5), 2)
            if rand_var >=0.3 and rand_var < 0.7:
                accel = round(random.uniform(-0.5, 0.5), 2)
            else:
                accel = round(random.uniform(0.5, 1.5), 2)
        accels.append(accel)
    return accels   

def generate_rand_phis(accels):
    phis= []
    for accel in accels:
        if accel == 0.0:
            phi = 0.0
        else:    
            phi = round(random.uniform(-0.5, 0.5), 2)
        phis.append(phi)
    return phis  
        
if __name__ == "__main__":

    bbx1 = [0.0, 0.0, 0.0, 4.0, 2.0, 0.8]
    bbx2 = [1.0, 2.0, math.pi/4, 3.0, 1.5, 0.7]
    bbx3 = [3.0, -1.0, -math.pi/2, 5.0, 3.0, 0.9]
    bbxs = [bbx1, bbx2, bbx3]

    v0s = [10.0, 15.0, 12.0]
    omegas = [0.1, 0.2, 0.15]
    accels = [1.0, 1.5, 1.2]
    fyis = [0.0, 0.0, 0.0]

    dt = 1

    # Move the vehicles one step into the future
    new_bbxs = ctrv_motion_model(bbxs, v0s, omegas, accels, fyis, dt)

    for i, new_bbx in enumerate(new_bbxs):
        x, y, yaw, l, w, dscore = new_bbx
        print(f"Vehicle {i + 1} - Current position: ({x}, {y}), Yaw angle: {yaw}")
        print(f"Next position: ({x}, {y}), Next yaw angle: {yaw}")
        print(f"Width: {w}, Length: {l}, Score: {dscore}")
        print()