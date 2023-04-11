#!/usr/bin/python
import cv2
import numpy

BAR_THICK = 2
BAR_COLOR_LEFT = (128,0,128)
BAR_COLOR_RIGHT = (50,205,50)
AR_COLOR_CENTER = (255,140,0)
BAR_COLOR_VERY_LEFT = (75,0,130)
BAR_COLOR_VERY_RIGHT = (0,128,0)

# Bar class
class Bar:
	def __init__(self):
		return

	def set_value(self,value):
		self.value = value

	def set_alpha(self,alpha):
		self.alpha = alpha
		

	def draw(self,canvas):
		bar_fill_start = (self.bar_start[0], int(self.bar_stop[1]-self.value*self.bar_height))
		tmp = canvas.copy()
		tmp = cv2.rectangle(tmp,bar_fill_start,self.bar_stop,self.color,-1) # Fill feedback bars
		cv2.addWeighted(tmp, self.alpha, canvas, 1 - self.alpha, 0, canvas)
		canvas = cv2.rectangle(canvas,self.bar_start,self.bar_stop,(255,255,255),BAR_THICK) # Draw feedback bars
		canvas = cv2.putText(canvas, self.name, (self.bar_stop[0] - self.bar_width,
                                                 self.bar_stop[1] + numpy.int32(self.bar_stop[1] / 8)),
                             20, 1, (255, 255, 255), 2, cv2.LINE_AA)    # Add text
		return canvas