#!/usr/bin/env python3

import os
import sys
import time
import math
import ev3dev2
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, SpeedPercent, MoveTank
from ev3dev2.sensor.lego import GyroSensor
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
    def __init__(self, fail_angle, n_states):
        self.gs = GyroSensor()
        time.sleep(0.5)
        c = self.gs.reset()
        self.starting_angle, self.starting_rate = self.gs.angle_and_rate
        self.fail_angle = fail_angle
        self.n_states = n_states
        #self.q_table = np.zeros((n_states, n_states, 2))

    def observation_to_state(self, angle, rate):
        a = math.floor(angle / (2 * self.fail_angle) * self.n_states)
        b = 0
        return a, b    

    def run_episode(self):
        # Wait until the robot is upright
        while 5 < abs(self.starting_angle - self.gs.angle()):
            print("Waiting "+str(self.starting_angle - self.gs.angle()))

        # Run until the robot isn't balanced
        while True:
            angle = self.starting_angle - self.gs.angle()
            rate = self.starting_rate - self.gs.rate()
            print("a "+str(self.gs.angle())+"\nR "+str(self.starting_angle))
            if self.fail_angle < abs(angle) and False:
                break

            a, b = self.observation_to_state(angle, rate)
        





def main():
    '''The main function of our program'''

    # set the console just how we want it
    reset_console()
    set_cursor(OFF)
    set_font('Lat15-Terminus24x12'),

    # Create balancer
    balancer = Balancer(45, 20)

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
