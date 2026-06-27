"""
import matplotlib.pyplot as plt
import random

# 定义道路宽度和车长
road_width = 20  # 假设道路宽度为20米
car_length = 5   # 假设车长为5米

# 创建一个十字路口的函数
def create_intersection():
    intersection = plt.Rectangle((-road_width/2, -road_width/2), road_width, road_width, color='gray')
    plt.gca().add_patch(intersection)

# 在十字路口上随机生成车辆点的函数
def generate_car_points(num_points):
    car_points = []
    for _ in range(num_points):
        x = random.uniform(-road_width/2 + car_length, road_width/2 - car_length)
        y = random.uniform(-road_width/2 + car_length, road_width/2 - car_length)
        car_points.append((x, y))
    return car_points

# 绘制程序
def main():
    plt.figure(figsize=(8, 8))
    create_intersection()

    num_points = 20  # 假设生成20个车辆点
    car_points = generate_car_points(num_points)

    for x, y in car_points:
        plt.plot(x, y, 'ro')

    plt.xlim(-road_width/2, road_width/2)
    plt.ylim(-road_width/2, road_width/2)
    plt.gca().set_aspect('equal', adjustable='box')
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.title('Intersection with Possible Car Points')
    plt.grid(True)
    plt.show()
"""
"""
import matplotlib.pyplot as plt
import random

# 定义道路宽度、车道宽度和车长
road_width = 20     # 假设道路宽度为20米
lane_width = 5      # 假设车道宽度为5米
car_length = 5      # 假设车长为5米

# 创建一个十字路口的函数
def create_intersection():
    intersection = plt.Rectangle((-road_width/2, -road_width/2), road_width, road_width, color='gray')
    plt.gca().add_patch(intersection)

    # 画车道线
    for i in range(-2, 3):
        plt.plot([-road_width/2, road_width/2], [i * lane_width, i * lane_width], 'k--')

        if i != 0:
            plt.plot([i * lane_width, i * lane_width], [-road_width/2, road_width/2], 'k--')

# 在十字路口上随机生成车辆点的函数
def generate_car_points(num_points):
    car_points = []
    for _ in range(num_points):
        x = random.uniform(-road_width/2 + car_length, road_width/2 - car_length)
        y = random.uniform(-road_width/2 + car_length, road_width/2 - car_length)
        car_points.append((x, y))
    return car_points

# 绘制程序
def main():
    plt.figure(figsize=(8, 8))
    create_intersection()

    num_points = 20  # 假设生成20个车辆点
    car_points = generate_car_points(num_points)

    for x, y in car_points:
        plt.plot(x, y, 'ro')

    plt.xlim(-road_width/2, road_width/2)
    plt.ylim(-road_width/2, road_width/2)
    plt.gca().set_aspect('equal', adjustable='box')
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.title('Intersection with Lane Lines and Possible Car Points')
    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    main()

"""
"""
import matplotlib.pyplot as plt
import random

# 定义道路宽度、车道宽度和车长
road_width = 50     # 假设道路宽度为20米
lane_width = 3.7      # 假设车道宽度为5米
car_length = 5      # 假设车长为5米

# 创建一个十字路口的函数
def create_intersection():
    intersection = plt.Rectangle((-road_width/2, -road_width/2), road_width, road_width, color='gray')
    plt.gca().add_patch(intersection)

    # 画道路轮廓线
    plt.plot([-road_width/2, road_width/2, road_width/2, -road_width/2, -road_width/2],
             [-road_width/2, -road_width/2, road_width/2, road_width/2, -road_width/2], 'k-')


    # 画车道线
    for i in range(-2, 3):
        plt.plot([-road_width/2, road_width/2], [i * lane_width, i * lane_width], 'k-')
        plt.plot([i * lane_width, i * lane_width], [-road_width/2, road_width/2], 'k-')

# 在十字路口上生成车辆位置的函数
def generate_car_positions0():
    car_positions = []
    for i in range(-1, 2):
        for j in range(-1, 2):
            if (i, j) != (0, 0):
                x = i * lane_width
                y = j * lane_width
                car_positions.append((x, y))
    return car_positions

# 在十字路口上生成车辆位置的函数
def generate_car_positions():
    car_positions = []
    for i in range(-1, 2):
        for j in range(-1, 2):
            if (i, j) != (0, 0):
                x = i * lane_width
                y = j * lane_width + random.uniform(-lane_width/2 + car_length, lane_width/2 - car_length)
                car_positions.append((x, y))
    return car_positions
# 绘制程序
def main():
    plt.figure(figsize=(8, 8))
    create_intersection()

    car_positions = generate_car_positions()

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
    main()
"""

import matplotlib.pyplot as plt
import random
import math
import sys

# 定义道路宽度、车道宽度和车长
road_width = 50      # 假设道路宽度为50米
lane_width = 3.7     # 假设车道宽度为3.7米
car_length = 5       # 假设车长为5米

total_car_num = 20

# 创建一个十字路口的函数
def create_intersection():
    # 画道路轮廓线
    plt.plot([-road_width/2, road_width/2, road_width/2, -road_width/2, -road_width/2],
             [-road_width/2, -road_width/2, road_width/2, road_width/2, -road_width/2], 'k-')

    # 画垂直方向车道线
    for i in range(-2, 3):
        plt.plot([-road_width/2, road_width/2], [i * lane_width, i * lane_width], 'k-')
        plt.plot([i * lane_width, i * lane_width], [-road_width/2, road_width/2], 'k-')

# 在道路上生成车辆位置的函数
def generate_car_positions(car_num):
    car_positions = []
    occupied_positions = set()

    # 随机生成车辆位置
    while len(car_positions) < car_num:  
        rand_value = random.random()
        """
        if rand_value > 0.5:
            x = random.uniform(-lane_width * 2 + car_length/2, lane_width * 2 - car_length/2)
            y = random.uniform(-road_width/2 + car_length, road_width/2 - car_length)
        else:
            x = random.uniform(-road_width/2 + car_length, road_width/2 - car_length)
            y = random.uniform(-lane_width * 2 + car_length/2, lane_width * 2 - car_length/2)
        """
        if rand_value < 0.25:
            x = random.uniform(lane_width/4, lane_width * 2 - car_length/4)
            y = random.uniform(-road_width/2 + car_length, road_width/2 - car_length)
            # check if two veh cross with each other
            is_valid = True
            for pos in occupied_positions:
                if abs(x - pos[0]) < (car_length-2) and abs(y - pos[1]) < car_length:
                    is_valid = False
                    break
            if is_valid:
                car_positions.append((x, y))
                occupied_positions.add((x, y))
                
        elif rand_value < 0.5 and rand_value >= 0.25:
            x = random.uniform(- lane_width * 2 + car_length/4 , - lane_width/4)
            y = random.uniform(-road_width/2 + car_length, road_width/2 - car_length)
            # check if two veh cross with each other
            is_valid = True
            for pos in occupied_positions:
                if abs(x - pos[0]) < (car_length-2) and abs(y - pos[1]) < car_length:
                    is_valid = False
                    break
            if is_valid:
                car_positions.append((x, y))
                occupied_positions.add((x, y))
                
        elif rand_value >= 0.5 and rand_value > 0.75:
            y = random.uniform(lane_width/4, lane_width * 2 - car_length/4)
            x = random.uniform(-road_width/2 + car_length, road_width/2 - car_length)
            # check if two veh cross with each other
            is_valid = True
            for pos in occupied_positions:
                if abs(x - pos[0]) < car_length and abs(y - pos[1]) < (car_length-2):
                    is_valid = False
                    break
            if is_valid:
                car_positions.append((x, y))
                occupied_positions.add((x, y))
                
        else:
            y = random.uniform(- lane_width * 2 + car_length/4 , - lane_width/4)
            x = random.uniform(-road_width/2 + car_length, road_width/2 - car_length)
            # check if two veh cross with each other
            is_valid = True
            for pos in occupied_positions:
                if abs(x - pos[0]) < car_length and abs(y - pos[1]) < (car_length-2):
                    is_valid = False
                    break
            if is_valid:
                car_positions.append((x, y))
                occupied_positions.add((x, y))
                
        # x = random.uniform(- lane_width * 2, lane_width * 2)
        # y = random.uniform(- lane_width * 2, lane_width * 2)
        # 检查是否与已有车辆位置相交
        is_valid = True
        for pos in occupied_positions:
            if abs(x - pos[0]) < car_length and abs(y - pos[1]) < (car_length-1):
                is_valid = False
                break
        if is_valid:
            car_positions.append((x, y))
            occupied_positions.add((x, y))

    return car_positions

# 绘制程序
def plot_intersection_and_pose():
    plt.figure(figsize=(10, 10))
    create_intersection()

    car_positions = generate_car_positions(total_car_num)

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

def generate_vehicle_dimensions(vehicle_type, quantity):
    dimensions = []

    for _ in range(quantity):
        if vehicle_type == "Sedan":
            length = round(random.uniform(4.8, 5.3), 2)
            width = round(random.uniform(1.8, 1.9), 2)
        elif vehicle_type == "SUV":
            length = round(random.uniform(4.6, 5.4), 2)
            width = round(random.uniform(1.8, 1.9), 2)
        elif vehicle_type == "Light Truck":
            length = round(random.uniform(5, 6), 2)
            width = round(random.uniform(1.8, 2.2), 2)
        elif vehicle_type == "Medium Truck":
            length = round(random.uniform(7, 9), 2)
            width = round(random.uniform(2.2, 2.6), 2)
        elif vehicle_type == "Heavy Truck":
            length = round(random.uniform(10, 15), 2)
            width = round(random.uniform(2.6, 3.0), 2)
        else:
            print("Invalid vehicle type")
            return
        
        dimensions.append([length, width])

    return dimensions

def generate_vehicle_dimensions_single(vehicle_type):
    dimensions = []

    if vehicle_type == "Sedan":
        length = round(random.uniform(4.8, 5.3), 2)
        width = round(random.uniform(1.8, 1.9), 2)
    elif vehicle_type == "SUV":
        length = round(random.uniform(4.6, 5.4), 2)
        width = round(random.uniform(1.8, 1.9), 2)
    elif vehicle_type == "Light Truck":
        length = round(random.uniform(5, 6), 2)
        width = round(random.uniform(1.8, 2.2), 2)
    elif vehicle_type == "Medium Truck":
        length = round(random.uniform(7, 9), 2)
        width = round(random.uniform(2.2, 2.6), 2)
    elif vehicle_type == "Heavy Truck":
        length = round(random.uniform(10, 15), 2)
        width = round(random.uniform(2.6, 3.0), 2)
    else:
        print("Invalid vehicle type")
        return

    return (length, width)

def vehicle_dimention_type_single(): 
    vehicle_type = "Sedan"  # 选择车辆类型，可选值为 "Sedan", "SUV", "Light Truck", "Medium Truck", "Heavy Truck"
    quantity = 5  # 生成的车辆数量

    vehicle_dimensions = generate_vehicle_dimensions(vehicle_type, quantity)
 
def vehicle_dimention_type_generator(car_num):
    vehicle_dimensions = []
    while len(vehicle_dimensions) < car_num:
        rand_var = random.random()
        if rand_var < 0.5:
            vehicle_type = "Sedan"  # vehicle type: "Sedan", "SUV", "Light Truck", "Medium Truck", "Heavy Truck"
            vehicle_dimension = generate_vehicle_dimensions_single(vehicle_type)
            vehicle_dimensions.append(vehicle_dimension)
        elif rand_var >= 0.5 and rand_var < 0.97:
            vehicle_type = "SUV"  
            vehicle_dimension = generate_vehicle_dimensions_single(vehicle_type)
            vehicle_dimensions.append(vehicle_dimension)
        elif rand_var >= 0.97 and rand_var < 0.98:
            vehicle_type = "Light Truck"  
            vehicle_dimension = generate_vehicle_dimensions_single(vehicle_type)
            vehicle_dimensions.append(vehicle_dimension)

        elif rand_var >= 0.98 and rand_var < 0.99:
            vehicle_type = "Medium Truck"  
            vehicle_dimension = generate_vehicle_dimensions_single(vehicle_type)
            vehicle_dimensions.append(vehicle_dimension)
        else:
            vehicle_type = "Heavy Truck"  
            vehicle_dimension = generate_vehicle_dimensions_single(vehicle_type)
            vehicle_dimensions.append(vehicle_dimension)
    return vehicle_dimensions

def generate_car_positions_yaw(car_positions):
    car_positions_yaw = []
    
    for x, y in car_positions:
        if (x < -lane_width * 2) or (x > lane_width * 2):
            yaw = math.radians(random.uniform(-15, 15))
        elif (y < -lane_width * 2) or (y > lane_width * 2):
            yaw = math.radians(random.uniform(75, 105))
        else:
            yaw = math.radians(random.uniform(-180, 180))
        
        car_positions_yaw.append((x, y, yaw))
    
    return car_positions_yaw

    
if __name__ == "__main__":
    vehicle_dimensions = vehicle_dimention_type_generator(total_car_num)
    car_positions = generate_car_positions(total_car_num)
    car_positions_yaw = generate_car_positions_yaw(car_positions)

