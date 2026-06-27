import numpy as np
import matplotlib.pyplot as plt
from vis import bbx_vis
from intersection import vehicle_dimention_type_generator,generate_car_positions,generate_car_positions_yaw
from intersection import  create_intersection
from motion_model import ctrv_motion_model,generate_rand_v0s,generate_rand_omegas,generate_rand_accels,generate_rand_phis


# define parameters
road_width = 50      # 50m
lane_width = 3.7     # 3.7m
car_length = 5       # 5m
total_car_num = 10
    
def load_saved_data(npy_name):

    # Specify the path to the .npy file
    # example here 
    # npy_name = "/home/zhaoliang/zzl/zhz03_github/CooperFuse/sim/sim_data/bbxs.npy"
    
    # Load the data from the .npy file
    loaded_data = np.load(npy_name) 
    
    return loaded_data

def plot_saved_data(bbxs):
    
    plt.figure(figsize=(10, 10))
    create_intersection()
    bbx_vis(bbxs, color='k', alpha=0.5)
    for x, y,yaw,l,w,dscore in bbxs:
        plt.plot(x, y, 'ro')
    plt.xlim(-road_width/2, road_width/2)
    plt.ylim(-road_width/2, road_width/2)
    plt.gca().set_aspect('equal', adjustable='box')
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.title('Intersection with Road Outline and Car Positions')
    plt.grid(True)
    plt.show()

def plot_saved_data_movement_v0(bbxs1,bbxs2):
    
    plt.figure(figsize=(10, 10))
    create_intersection()
    bbx_vis(bbxs1, color='k', alpha=0.5)
    bbx_vis(bbxs2, color='b', alpha=0.9)
    for x, y,yaw,l,w,dscore in bbxs1:
        plt.plot(x, y, 'ro')
    plt.xlim(-road_width/2, road_width/2)
    plt.ylim(-road_width/2, road_width/2)
    plt.gca().set_aspect('equal', adjustable='box')
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.title('Intersection with Road Outline and Car Positions')
    plt.grid(True)
    plt.show()

def plot_saved_data_movement(bbxs1,bbxs2):
    
    plt.figure(figsize=(10, 10))
    create_intersection()
    bbx_vis(bbxs1, color='k', alpha=0.5)
    bbx_vis(bbxs2, color='b', alpha=0.9)
    for vid,x, y,yaw,l,w,dscore,ss,kds in bbxs1:
        plt.plot(x, y, 'ro')
    plt.xlim(-road_width/2, road_width/2)
    plt.ylim(-road_width/2, road_width/2)
    plt.gca().set_aspect('equal', adjustable='box')
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.title('Intersection with Road Outline and Car Positions')
    plt.grid(True)
    plt.show()
    
def convert_data(car_positions_yaw,vehicle_dimensions):
    bbxs = []
    dscore = 0
    ss = 0
    kds = 0
    vid = 0
    for (x, y, yaw), (length, width) in zip(car_positions_yaw, vehicle_dimensions):
        vid += 1
        bbx = [vid,x, y, yaw, length, width, dscore,ss,kds]
        bbxs.append(bbx)
    
    return bbxs 

def generate_bbxs_on_intersection():
    vehicle_dimensions = vehicle_dimention_type_generator(total_car_num)
    car_positions = generate_car_positions(total_car_num)
    car_positions_yaw = generate_car_positions_yaw(car_positions)
    
    
    bbxs = convert_data(car_positions_yaw,vehicle_dimensions)
    
    # save data
    # np.save("/home/zhaoliang/zzl/zhz03_github/CooperFuse/sim/sim_data/new_bbxs.npy", bbxs)
    npy_name = "/home/zhaoliang/zzl/zhz03_github/CooperFuse/sim/sim_data/bbxs_2.npy"
    np.save(npy_name, bbxs)
    
    plt.figure(figsize=(10, 10))
    create_intersection()
    
    bbx_vis(bbxs, color='b', alpha=0.5)
    
    for x, y in car_positions:
        plt.plot(x, y, 'ro')
    plt.xlim(-road_width/2, road_width/2)
    plt.ylim(-road_width/2, road_width/2)
    plt.gca().set_aspect('equal', adjustable='box')
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.title('Intersection with Road Outline and Car Positions')
    plt.grid(True)
    plt.show()    
if __name__ == "__main__":
    pass
    """
    # plot_saved_data()
    bbxs = load_saved_data()
    car_num = len(bbxs)
    v0s = generate_rand_v0s(car_num)
    omegas = generate_rand_omegas(v0s)
    accels = generate_rand_accels(omegas)
    phis = generate_rand_phis(accels)
    
    
    dt = 0.1
    new_bbxs = ctrv_motion_model(bbxs, v0s, omegas, accels, phis, dt)
    # plot_saved_data(bbxs)
    # plot_saved_data(new_bbxs)
    
    plot_saved_data_movement(bbxs,new_bbxs)
    """

