#!/usr/bin/env python3

import os
import sys
import time
import math
import random
from time import sleep
import ev3dev2
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, SpeedPercent, MoveTank
from ev3dev2.sensor.lego import UltrasonicSensor
from ev3dev2.sensor.lego import ColorSensor
from ev3dev.ev3 import *
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
    def __init__(self, fail_distance_max, fail_distance_min, n_states):
        self.us = UltrasonicSensor()
        self.ir = ColorSensor()

        time.sleep(0.5)
        self.starting_distance_ir = self.ir.reflected_light_intensity
        #self.starting_distance = self.us.distance_centimeters
        #self.fail_distance_ir = fail_distance
        self.fail_distance_max = fail_distance_max
        self.fail_distance_min = fail_distance_min
        self.n_states = n_states
        ## TODO: Other values than zero. Random values perhaps.
        self.q_table = [[random.random(), random.random()] for i in range(0, n_states)]

    def observation_to_state(self, distance):
        a = math.floor(((distance - self.fail_distance_min) / (self.fail_distance_max - self.fail_distance_min)) * self.n_states)
        if a == self.n_states:
            a -= 1
        b = 0
        if a < 0:
            a = 0
        elif a > self.n_states:
            a = self.n_states - 1
        return a, b

    def find_best_action(self, a):
        max_val = -100000
        action = 0
        for i in range(len(self.q_table[a])):
            if max_val < self.q_table[a][i]:
                action = i
                max_val = self.q_table[a][i]
        if action == 0:
            debug_print("0: "+str(self.q_table[a][0])+" "+str(self.q_table[a][1]))
        else:
            debug_print("1: "+str(self.q_table[a][0])+" "+str(self.q_table[a][1]))
        return action, max_val

    def run_episode(self):
        sound = Sound()
        #self.us.distance_centimeters
        while False:
            debug_print(self.starting_distance_ir - self.ir.reflected_light_intensity)
            sound.tone(100 * abs(self.starting_distance_ir - self.ir.reflected_light_intensity)+100, 50)
            sleep(0.5)
        
        # Wait until the robot is upright
        while 3 < abs(self.starting_distance_ir - self.ir.reflected_light_intensity):
            print("Waiting "+str(round(self.starting_distance_ir - self.ir.reflected_light_intensity, 2)))

       
        # Run until the robot isn't balanced
        while True:
            #sleep(1)
            distance = self.starting_distance_ir - self.ir.reflected_light_intensity
            
            print("D "+str(self.ir.reflected_light_intensity))
            if self.fail_distance_min > distance or self.fail_distance_max < distance:
                sound.tone(200, 200).wait()
                
                break

            a, b = self.observation_to_state(distance)
            debug_print(str(a) +"  "+ str(distance))



            if random.random() < 0.15:
                action = random.randint(0, 1)
                value = self.q_table[a][action]
            else:
                action, value = self.find_best_action(a)
                
            
            # TODO: tell motor
            motor = LargeMotor('outA')
            
            #if action == 1:
            #    motor.run_timed(time_sp = 250, speed_sp = -120, stop_action = 'brake')
            #else:
            #    motor.run_timed(time_sp = 250, speed_sp = 120, stop_action = 'brake')
        
            # Wait for physics to do its thang

            eta = 0.1
            gamma = 0.2
            
            new_distance = self.starting_distance_ir - self.ir.reflected_light_intensity
            reward = -abs(new_distance) + 4
            a_, b_ = self.observation_to_state(new_distance)

            new_best_action, new_value = self.find_best_action(a_)
            self.q_table[a][action] += eta * (reward + gamma *  new_value - value)
            sound.tone(-200 * (reward-4)+100,50)
            #debug_print(self.q_table[a][action])







def main():
    '''The main function of our program'''

    # set the console just how we want it
    reset_console()
    set_cursor(OFF)
    set_font('Lat15-Terminus24x12'),

    # Create balancer
    balancer = Balancer(6, -13, 15)

    while True:
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
