#!/usr/bin/python
import cv2
import rospy
import rospkg
import os
import random
import math
import numpy
import time
from rosneuro_msgs.msg import NeuroEvent, NeuroOutput
from std_srvs.srv import Empty
#from draw.GUI import SMRGUI
from draw.GUI_WHEEL import SMRGUI_WHEEL
from draw.GUI_DWHEEL import SMRGUI_DWHEEL



# TODO: Export to rosneuro_events.yaml
OFF = 32768
TARGETMASK = 24576
FIXATION = 786
CFEEDBACK = 781
CLASS_EVENTS = [773, 771, 783]
TARGETHIT = 897
TARGETMISS = 898
EOG = 1024
TIMEOUT = 899

# TODO: Export to smr_protocol.yaml
CLASSES = ['mi_both_hands', 'mi_both_feet', 'rest']

def config_trials_rest(n_classes, n_trials):
	sequence = []
	for i in range(n_classes):
		sequence.extend([i]*n_trials)
	del sequence[-int(n_trials/2):]
	sequence.extend([3]*int(n_trials/2))
	random.shuffle(sequence)
	return sequence
	
def config_trials(n_classes,n_trials):
	sequence = []
	for i in range(n_classes):
		sequence.extend([i]*n_trials)
	random.shuffle(sequence)
	return sequence

def check_exit(key_pressed):
	if key_pressed is -1: return False
	if chr(key_pressed) is 'q': return True

def publish_neuro_event(pub,event):
	msg = NeuroEvent()
	msg.header.stamp = rospy.Time.now()
	msg.event = event
	pub.publish(msg)

def normalize_probabilities(value,max,min):
	nvalue = ((1.0 - 0.0) * (value - min))/(max - min) + 0.0
	nvalue = 1.0 if nvalue >= 1.0 else nvalue
	nvalue = 0.0 if nvalue <= 0.0 else nvalue
	return nvalue

def normalize_probabilities_wheel_bar(value, newMax, newMin, oldMax, oldMin):
	nvalue = ((value - oldMin) * (newMax - newMin))/(oldMax - oldMin) + newMin
	nvalue = 1.0 if nvalue >= 1.0 else nvalue
	nvalue = -1.0 if nvalue <= -1.0 else nvalue
	return nvalue

def normalize_th(value, newMax, newMin, oldMax, oldMin):
	nvalue = ((value - oldMin) * (newMax - newMin))/(oldMax - oldMin) + newMin
	nvalue = 1.0 if nvalue >= 1.0 else nvalue
	nvalue = -1.0 if nvalue <= -1.0 else nvalue
	return nvalue

def check_boom(values):
	if any(i >= 1.0 for i in values): return True
	else: return False

class feedback_timecheck:
	def __init__(self, limit=None):
		self.tic = 0
		self.elapsed = 0
		self.limit = limit

	def make_tic(self):
		self.tic = time.time()

	def make_toc(self):
		self.elapsed = (time.time()-self.tic)*1000
		return self.elapsed

	def check_delay(self):
		if self.limit != None and self.elapsed > self.limit:
			print('[feedback_timecheck] WARNING! The feedback update timing is too low (delay ' + str(self.elapsed-self.limit) + ' ms)')

		return int(self.elapsed - self.limit)

