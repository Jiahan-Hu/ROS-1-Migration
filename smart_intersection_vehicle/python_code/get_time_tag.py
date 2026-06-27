"""
This code is to get the time tag of the data by pressing the keyboard.
Author: Zhaoliang Zheng
"""

import time
import pygame
import os

def init():
    pygame.init()
    win= pygame.display.set_mode((200,200))

def get_key(keyname):
    ans = False
    for eve in pygame.event.get(): pass
    keyInput = pygame.key.get_pressed()
    myKey = getattr(pygame,'K_{}'.format(keyname))

    if keyInput[myKey]:
        ans = True
    pygame.display.update()
    return ans

def print_tips():
    print("---------------Tips:--------------------")
    print("Press (n) to create a new file and begin recording")
    print("Press (s) to start the scenario")
    print("Press (e) to end the scenario")
    print("Press (p) to tag the pedestrian scenario")
    print("Press (b) to tag the bus scenario")
    print("Press (g) to tag the scooter scenario")
    print("Press (q) to quit the program")
    print("-----------------------------------------")
if __name__ == '__main__':
    init()
    print_count = 0
    print_tips()
    # create a directory to save the data
    # if the directory is not exist, create it
    # if the directory is exist, do nothing
    os.makedirs('data',exist_ok=True)
    while True:
        if get_key('n'):
            # begin tagging to a file
            print('create a new file')
            # use the time tag as the file name
            time_tag = time.strftime('%Y-%m-%d-%H-%M-%S',time.localtime(time.time()))
            print(time_tag, ".txt is recording")
            flag = 1
            with open("./data/" + time_tag + '.txt','w') as f:
                while (flag==1):
                    if get_key('q'):
                        # end tagging
                        print('end tagging')
                        flag = 0
                        # close the file
                        f.close()
                        print_count = 1
                        time.sleep(0.5)
                    elif get_key('s'):
                        # get the time tag
                        event_time_tag = time.strftime('%Y-%m-%d-%H-%M-%S',time.localtime(time.time()))
                        event = 'Scenario starts:'
                        tag_all =  event + event_time_tag
                        print(tag_all)
                        f.write(tag_all)
                        f.write('\n')
                        time.sleep(0.5)
                    elif get_key('e'):
                        # get the time tag
                        event_time_tag = time.strftime('%Y-%m-%d-%H-%M-%S',time.localtime(time.time()))
                        event = 'Scenario end:'
                        tag_all =  event + event_time_tag
                        print(tag_all)
                        f.write(tag_all)
                        f.write('\n')
                        time.sleep(0.5)
                    elif get_key('p'):
                        # get the time tag
                        event_time_tag = time.strftime('%Y-%m-%d-%H-%M-%S',time.localtime(time.time()))
                        event = 'Pedestrains:'
                        tag_all =  event + event_time_tag
                        print(tag_all)
                        f.write(tag_all)
                        f.write('\n')
                        time.sleep(0.5)
                    elif get_key('b'):
                        # get the time tag
                        event_time_tag = time.strftime('%Y-%m-%d-%H-%M-%S',time.localtime(time.time()))
                        event = 'Bus:'
                        tag_all =  event + event_time_tag
                        print(tag_all)
                        f.write(tag_all)
                        f.write('\n')
                        time.sleep(0.5)
                    elif get_key('g'):
                        # get the time tag
                        event_time_tag = time.strftime('%Y-%m-%d-%H-%M-%S',time.localtime(time.time()))
                        event = 'Scooter:'
                        tag_all =  event + event_time_tag
                        print(tag_all)
                        f.write(tag_all)
                        f.write('\n')
                        time.sleep(0.5)
                    elif get_key('h'):
                        print_tips()
                        print("The above content within ------ will not be recorded")
                        time.sleep(0.5)
        elif get_key('q'):
            # quit the program
            print('quit_program')
            break

        if print_count != 0:
            print("No txt file is recording...")
            print("Please press (n) to begin a new file")
            print_count = 0
            
                        

            