#!/usr/bin/env python3

import os
import sys
import time
import math
import random
import ev3dev2
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, SpeedPercent, MoveTank
from ev3dev2.sensor.lego import UltrasonicSensor
# state constants
ON = True
OFF = False


def debug_print(*args, **kwargs):
    '''Print debug messages to stderr.

    This shows up in the output panel in VS Code.
    '''
    print(*args, **kwargs, file=sys.stderr)


def reset_console():
    '''Resets the console to the default state'''
    print('\x1Bc', end='')


def set_cursor(state):
    '''Turn the cursor on or off'''
    if state:
        print('\x1B[?25h', end='')
    else:
        print('\x1B[?25l', end='')


def set_font(name):
    '''Sets the console font

    A full list of fonts can be found with `ls /usr/share/consolefonts`
    '''
    os.system('setfont ' + name)


""" Learning balancer class """
class Balancer:
    def __init__(self, fail_distance, n_states):
        self.us = UltrasonicSensor()
        time.sleep(0.5)
        self.starting_distance = self.us.distance_centimeters
        self.fail_distance = fail_distance
        self.n_states = n_states
        
        self.q_table = [[0, 0] for i in range(0, n_states)]

    def observation_to_state(self, distance):
        a = math.floor((distance / (2 * self.fail_distance) + 0.5) * self.n_states)
        b = 0
        return a, b

    def find_best_action(a):
        max_val = -100000
        action = 0
        for i in range(len(self.q_table[a])):
            if max_val < self.q_table[a][i]:
                action = i
                max_val = self.q_table[a][i]
        return action, max_val

    def run_episode(self):
        # Wait until the robot is upright
        while 2 < abs(self.starting_distance - self.us.distance_centimeters):
            print("Waiting "+str(self.starting_distance - self.us.distance_centimeters))

        # Run until the robot isn't balanced
        while True:
            distance = self.starting_distance - self.us.distance_centimeters
            
            print("D "+str(self.us.distance_centimeters))
            if self.fail_distance < abs(distance):
                break

            a, b = self.observation_to_state(distance)

            if random.random() < 0.2:
                action = random.randint(0, 1)
                value = self.q_table[a][action]
            else:
                action, value = self.find_best_action(a)
                
            
            # TODO: tell motor
            # Wait for phisics to do its thang

            eta = 0.5
            gamma = 0.5
            
            new_distance = self.starting_distance - self.us.distance_centimeters
            reward = -abs(new_distance)
            a_, b_ = self.observation_to_state(new_distance)

            new_best_action, new_value = self.find_best_action(a_)
            q_table[a][action] += eta * (reward + gamma *  new_value - value])







def main():
    '''The main function of our program'''

    # set the console just how we want it
    reset_console()
    set_cursor(OFF)
    set_font('Lat15-Terminus24x12'),

    # Create balancer
    balancer = Balancer(10, 10)

    for i in range(100):
        balancer.run_episode()

    # print something to the screen of the device
    print('ALL YOUR BASE ARE BELONG TO US')
    
    # print something to the output panel in VS Code
    debug_print('Hello VS Code!')

    # wait a bit so you have time to look at the display before the program
    # exits
    time.sleep(1)

if __name__ == '__main__':
    main()
