#!/usr/bin/python3
from smr_calibration_wheel import *
from smr_evaluation_wheel import *
from smr_control_wheel import *
from smr_control_continuous_wheel import *
import sys
import rospy
import rospkg
import os



def get_mode(idx):
	modality={
			0:'smrCalibrationWheel',
			1:'smrEvaluationWheel',
			2:'smrControlWheel',
			3:'SmrControlContinuousWheel'
		}
	return modality.get(idx,"Unexpected protocol index")

def main():
	sys.argv = rospy.myargv(sys.argv)
	
	rospy.init_node('rosneuro_feedback')
	mode = get_mode(rospy.get_param('~protocol_mode'))


	if mode == 'smrEvaluationWheel':
		o = SmrEvaluationWheel()
	elif mode == 'smrCalibrationWheel':
		o = SmrCalibrationWheel()
	elif mode == 'smrControlWheel':
		o = SmrControlWheel()
	elif mode == 'SmrControlContinuousWheel':
		o = SmrControlContinuousWheel()
	else:
		print('[rosneuro_feedback] Unexpected protocol mode')
		return

	o.run()


if __name__ == '__main__':
	main()
