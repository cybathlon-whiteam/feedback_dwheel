#!/usr/bin/python
import cv2
import numpy

from draw.Bar import *

COLOR_TH = (70,70,70)

class SmrBar(Bar):
	def __init__(self,barWidth,relativePosition,windowCenter,wheelRadius,name=None):
		self.bar_width = numpy.int32(barWidth)
		bar_height_start = windowCenter[1]-int(wheelRadius/2)
		bar_height_stop = windowCenter[1]+wheelRadius
		self.bar_height = bar_height_stop-bar_height_start
		self.count_reset_text = 0

		distanceTimes = 2

		if relativePosition == 'left':
			bar_width_stop = windowCenter[0]-wheelRadius-distanceTimes*barWidth
			bar_width_start = bar_width_stop-barWidth
			self.color = BAR_COLOR_LEFT

		elif relativePosition == 'right':
			bar_width_start = windowCenter[0]+wheelRadius+distanceTimes*barWidth
			bar_width_stop = bar_width_start+barWidth
			self.color = BAR_COLOR_RIGHT

		elif relativePosition == 'very-left':
			bar_width_stop = windowCenter[0]-wheelRadius-2*distanceTimes*barWidth
			bar_width_start = bar_width_stop-barWidth
			self.color = BAR_COLOR_LEFT

		elif relativePosition == 'very-right':
			bar_width_start = windowCenter[0]+wheelRadius+2*distanceTimes*barWidth
			bar_width_stop = bar_width_start+barWidth
			self.color = BAR_COLOR_VERY_RIGHT

		else:
			print("Invalid bar type")
			bar_height_start = 0
			bar_height_stop = 0
			bar_width_start = 0
			bar_width_stop = 0
			self.color = (0,0,0)

		self.bar_start = (bar_width_start,bar_height_start)
		self.bar_stop = (bar_width_stop,bar_height_stop)

		self.alpha = 0.5
		self.value = 0.0
		self.name = name
		self.thresholds = []

	def set_thresholds(self,thresholds):
		self.thresholds.append(float(thresholds))

	def position_threshold(self,threshold):
		th_height = int(self.bar_stop[1] - threshold*self.bar_height)
		return [(self.bar_start[0], th_height), (self.bar_stop[0], th_height)]


	def draw(self,actual_canvas):
		canvas = actual_canvas.copy()
		print(self.value)
		bar_fill_start = (self.bar_start[0], int(self.bar_stop[1]-self.value*self.bar_height))
		
		tmp = cv2.rectangle(canvas,bar_fill_start,self.bar_stop,self.color,-1) # Fill feedback bars
		cv2.addWeighted(tmp, self.alpha, canvas, 1 - self.alpha, 0, canvas)

		canvas = cv2.rectangle(canvas,self.bar_start,self.bar_stop,(255,255,255),BAR_THICK) # Draw feedback bars

		coordinates_text = (self.bar_start[0], self.bar_start[1] - int(self.bar_width/2))

		if self.value > 0.98:
			self.count_reset_text = 10	# Count to draw the letter colored

		if self.count_reset_text > 0:	# Use the counter to draw the colored letter
			canvas = cv2.putText(canvas, self.name, coordinates_text, 20, 1.1, self.color, 2, cv2.LINE_AA)    # Add text
			self.count_reset_text -= 1
		else:
			canvas = cv2.putText(canvas, self.name, coordinates_text, 20, 1.1, (255, 255, 255), 2, cv2.LINE_AA) 


		for th in self.thresholds:
			pos_th = self.position_threshold(th)
			canvas = cv2.line(canvas, pos_th[0], pos_th[1], (255, 255, 255), BAR_THICK) 

		return canvas
