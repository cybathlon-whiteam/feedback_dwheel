#!/usr/bin/python
import cv2
import numpy
import math

COLOR_WHEEL_BIGGER       = (200,200,200)
COLOR_WHEEL_BAR_GREY     = (100,100,100)
COLOR_WHEEL_THS_DYNAMIC  = (50,50,50)
# COLOR_TH_LEFT           = (128,0,128)
# COLOR_TH_RIGHT          = (50,205,50)
COLOR_TH                 = (70,70,70)


# Wheel class
class Wheel:
    def __init__(self, canvas, center_coordinates, thickness, radious, angle_min = 0, angle_max = 180, name=None):
        # initial values
        self.canvas = canvas
        self.center_coordinates = center_coordinates
        self.thickness = thickness
        self.radious = radious
        self.value = 0.0
        self.name = name
        self.wheel_size = 100.0 # size of the wheel
        self.middle_angle = 90 # initialization is at 90 degree
        self.canvas = cv2.circle(self.canvas, self.center_coordinates, self.radious, COLOR_WHEEL_BIGGER, self.thickness) # draw the bigger wheel
        self.angle_min = angle_min 
        self.angle_max = angle_max 
        self.color_wheel = COLOR_WHEEL_BAR_GREY

    def set_th_left(self, th_left):
        self.th_left = th_left

    def set_th_right(self, th_right):
        self.th_right = th_right

    def set_angle_min(self, angle_min):
        self.angle_min = float(angle_min)

    def set_angle_max(self, angle_max):
        self.angle_max = float(angle_max)

    def set_value(self, value):
        self.value = value
	
    def set_angle_middle(self, middle_angle):
        self.middle_angle = middle_angle

    def set_color_wheel_bar(self, color = COLOR_WHEEL_BAR_GREY):
        self.color_wheel = color

    def draw(self):
        # the new middle wheel position is calculated by mapping the value in a range defined by angle_min and angle_max.
        #    in addition, the magic numbers are choosen because we now that the self.value stays in [-1, 1]
        middle_move = float((self.value + 1.0)*(self.angle_max - self.angle_min)/2.0 + self.angle_min) 

        wheel_bar_move = (float(middle_move - self.wheel_size/2.0), float(middle_move + self.wheel_size/2.0))
        tmp = self.canvas.copy()

        # draw in the new position the bar
        tmp1 = cv2.ellipse(tmp, self.center_coordinates, (self.radious + self.thickness//2, self.radious + self.thickness//2), 180, 
            wheel_bar_move[0], wheel_bar_move[1], self.color_wheel, -1)
        #tmp1 = cv2.ellipse(tmp, (0,0), (5,5), 180.0, 
        #    0.0, 45.0, (255,255,255), -1)
        cv2.addWeighted(tmp, 1.2, tmp1, 0.2, 0, tmp)
        tmp = cv2.circle(tmp, self.center_coordinates, int(self.radious - self.thickness/2), (0,0,0), -1) # for the inside circle


        # define the points: 
        # 	- central line is defined by q1 and q2
        #	- right line is defined by r1 and r2
        # 	- left line is defined by l1 and l2
        current_middle_angle = float((wheel_bar_move[0] + wheel_bar_move[1])/2) 
        q1 = (int(self.center_coordinates[0] - math.cos(math.pi*current_middle_angle/180)*(self.radious - self.thickness/2)), 
            int(self.center_coordinates[1] - math.sin(math.pi*current_middle_angle/180)*(self.radious - self.thickness/2))) 
        q2 = (int(self.center_coordinates[0] - math.cos(math.pi*current_middle_angle/180)*(self.radious + self.thickness/2)), 
            int(self.center_coordinates[1] - math.sin(math.pi*current_middle_angle/180)*(self.radious + self.thickness/2))) 
        
        l1 = (int(self.center_coordinates[0] - math.cos(math.pi*self.th_left/180)*(self.radious - self.thickness/2)), 
            int(self.center_coordinates[1] - math.sin(math.pi*self.th_left/180)*(self.radious - self.thickness/2)))
        l2 = (int(self.center_coordinates[0] - math.cos(math.pi*self.th_left/180)*(self.radious + self.thickness/2)), 
            int(self.center_coordinates[1] - math.sin(math.pi*self.th_left/180)*(self.radious + self.thickness/2)))
        r1 = (int(self.center_coordinates[0] - math.cos(math.pi*self.th_right/180)*(self.radious - self.thickness/2)), 
            int(self.center_coordinates[1] - math.sin(math.pi*self.th_right/180)*(self.radious - self.thickness/2)))
        r2 = (int(self.center_coordinates[0] - math.cos(math.pi*self.th_right/180)*(self.radious + self.thickness/2)), 
            int(self.center_coordinates[1] - math.sin(math.pi*self.th_right/180)*(self.radious + self.thickness/2)))

        # draw the line
        tmp = cv2.line(tmp, l1,l2, COLOR_TH, 2) 
        tmp = cv2.line(tmp, r1,r2, COLOR_TH, 2)
        tmp = cv2.line(tmp, q1,q2, (255,255,255), 2)



        # update angles
        self.set_angle_middle(current_middle_angle)

        return tmp
