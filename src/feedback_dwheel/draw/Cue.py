#!/usr/bin/python
import cv2
import numpy

CUE_COLOR_LEFT = (128,0,128)
CUE_COLOR_RIGHT = (50,205,50)
CUE_COLOR_CENTER = (255,140,0)
CUE_COLOR_VERY_LEFT = (75,0,130)
CUE_COLOR_VERY_RIGHT = (0,128,0)
CUE_COLOR_EOG = (0,0,150)

# Cue class
class Cue:
	def __init__(self,window,idx):
		self.center = (numpy.int32(window.width/2), numpy.int32(window.height/2 + window.height/8))
		self.size = numpy.int32(window.height/16)
		self.type = self.cue_type(idx)

	def cue_type(self,i):
		switcher={
			0:'left',
			1:'right',
			2:'center',
			3:'very-left',
			4:'very-right',
			5:'eog'
		}
		return switcher.get(i,"Invalid cue number")

	def set_center(self, center):
		self.center = center

	def draw(self,canvas):
		if self.type == 'left':
			color = CUE_COLOR_LEFT
		elif self.type == 'right':
			color = CUE_COLOR_RIGHT
		elif self.type == 'center':
			color = CUE_COLOR_CENTER
		elif self.type == 'very-left':
			color = CUE_COLOR_VERY_LEFT
		elif self.type == 'very-right':
			color = CUE_COLOR_VERY_RIGHT
		elif self.type == 'eog':
			color = CUE_COLOR_EOG
		else:
			print("Invalid cue type")
			color = (0,0,0)
		# Draw cue circle
		canvas = cv2.circle(canvas,self.center,self.size,color,-1)
		return canvas
