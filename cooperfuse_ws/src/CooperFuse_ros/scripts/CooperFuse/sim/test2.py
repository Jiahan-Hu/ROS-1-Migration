import numpy as np
import matplotlib.pyplot as plt
from vis import bbx_vis
from intersection import vehicle_dimention_type_generator,generate_car_positions,generate_car_positions_yaw
from intersection import  create_intersection
from intersection_bbx import convert_data

# define parameters
road_width = 50      # 50m
lane_width = 3.7     # 3.7m
car_length = 5       # 5m

total_car_num = 10
 
if __name__ == "__main__":
    vehicle_dimensions = vehicle_dimention_type_generator(total_car_num)
    car_positions = generate_car_positions(total_car_num)
    car_positions_yaw = generate_car_positions_yaw(car_positions)
    
    
    bbxs = convert_data(car_positions_yaw,vehicle_dimensions)
    
    # save data
    npy_name = "/Users/zhaoliang/Documents/Smart_intersection/CooperFuse/sim/sim_data/bbxs_2f.npy"
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
