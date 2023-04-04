#!/usr/bin/python
import cv2
import numpy 
from draw.Window import Window
from draw.DWheel import DWheel
from draw.Cue import Cue
from draw.Fixation import Fixation


# SMRGUI class
class SMRGUI_DWHEEL:
    def __init__(self, win_height, win_width, win_scale):
        cv2.namedWindow('canvas', cv2.WINDOW_NORMAL)
        cv2.moveWindow('canvas', 0, 0)
        self.window = Window(win_height,win_width,win_scale)
        self.canvas = numpy.zeros((win_height, win_width, 3), numpy.uint8)
        self.cue = []
        self.fixation = []
        # initialization for the variables used for canvas
        if self.window.width > self.window.height:
            self.r_bigger = int(self.window.height/2.5)
            self.thickness =  int(self.window.height*0.1)
        else:
            self.r_bigger = int(self.window.width/2.5)
            self.thickness =  int(self.window.width*0.1)
        self.r_arch = self.r_bigger
        self.center_coordinates = (int(self.window.width/2), int(self.window.height/2))
        self.wheel_bar = DWheel(self.canvas, self.center_coordinates, self.thickness, self.r_arch)

    def init_canvas(self):
        self.canvas = numpy.zeros((self.window.height, self.window.width, 3), numpy.uint8)

    def init_wheel_bar(self):
        del self.wheel_bar
    
        self.wheel_bar = DWheel(self.canvas, self.center_coordinates, self.thickness, self.r_arch)


    def set_th_left(self, th_left):
        self.wheel_bar.set_th_left(th_left)

    def set_ith_left(self, ith_left):
        self.wheel_bar.set_ith_left(ith_left)

    def set_yaw(self, yaw):
        self.wheel_bar.set_yaw(yaw)


    def set_th_right(self, th_right):
        self.wheel_bar.set_th_right(th_right)

    def set_ith_right(self, ith_right):
        self.wheel_bar.set_ith_right(ith_right)


    def set_value_wheel_bar(self, value):
        self.wheel_bar.set_value(value)

        self.draw()

    def set_color_wheel_bar(self, color):
        self.wheel_bar.set_color_wheel_bar(color)

        self.draw()

    def get_middle_angle_wheel_bar(self):
        return self.wheel_bar.get_middle_angle()

    def reset_wheel_bar(self):
        self.wheel_bar.set_value(0.0)
        self.wheel_bar.set_angle_middle(90)
        self.wheel_bar.set_color_wheel_bar()
        
        self.draw()

    def add_cue(self,idx):
        self.cue.append(Cue(self.window,idx))
        self.cue[-1].set_center(self.center_coordinates)
        self.draw()

    def add_fixation(self):
        self.fixation.append(Fixation(self.window))
        self.fixation[-1].set_center(self.center_coordinates)
        self.draw()

    def remove_cue(self):
        del self.cue[:]
        self.draw()

    def remove_fixation(self):
        del self.fixation[:]
        self.draw()

    def draw(self):

        self.canvas = self.wheel_bar.draw()

        for cue in self.cue:
            self.canvas = cue.draw(self.canvas)

        for fixation in self.fixation:
            self.canvas = fixation.draw(self.canvas)

        canvas = cv2.resize(self.canvas, (self.window.width * self.window.scale, self.window.height * self.window.scale))
		
        cv2.imshow('canvas', canvas)

