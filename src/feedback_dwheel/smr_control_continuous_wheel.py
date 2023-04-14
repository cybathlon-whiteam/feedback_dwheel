#!/usr/bin/python

from smr_utilities import *

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

class SmrControlContinuousWheel(object):
    def __init__(self):

        ##### Configure publisher #####
        self.event_pub = rospy.Publisher("/events/bus", NeuroEvent, queue_size=1000)

        ##### Configure subscriber #####
        rospy.Subscriber("/integrator/neuroprediction", NeuroOutput, self.receive_probabilities)
        rospy.Subscriber("/cmd_vel", Twist, self.receive_yaw)
        rospy.Subscriber('/bar_status', Float32MultiArray, self.receive_values_bars)


        ##### Configure protocol #####
        self.n_classes = rospy.get_param('~n_classes')
        self.n_trials = rospy.get_param('~n_trials')
        
        str_thr = rospy.get_param('~thresholds_soft') 
        list_thr = str_thr.split(",")
        self.list_thrs = [normalize_th(1-float(list_thr[0]), 1, -1, 1, 0), normalize_th(float(list_thr[1]), 1, -1, 1, 0)]
        self.threshold_angles =  [(self.list_thrs[0] + 1)*180/2, (self.list_thrs[1] + 1)*180/2]
        
        str_ithr = rospy.get_param('~thresholds_hard') 
        list_ithr = str_ithr.split(",")
        self.list_ithrs = [normalize_th(1-float(list_ithr[0]), 1, -1, 1, 0), normalize_th(float(list_ithr[1]), 1, -1, 1, 0)]
        self.threshold_iangles =  [(self.list_ithrs[0] + 1)*180/2, (self.list_ithrs[1] + 1)*180/2]


        self.colors = [(128,0,128), (50,205,50)]

        self.values = numpy.zeros(self.n_classes)
        self.rec_prob = numpy.zeros(self.n_classes)

        self.timings_begin = rospy.get_param('~timings_begin')
        self.timings_fixation = rospy.get_param('~timings_fixation')
        self.timings_cue = rospy.get_param('~timings_cue')
        self.timings_feedback_update = rospy.get_param('~timings_feedback_update')
        self.timings_boom = rospy.get_param('~timings_boom')
        self.timings_iti = rospy.get_param('~timings_iti')
        self.timings_end = rospy.get_param('~timings_end')

        self.time_checker = feedback_timecheck(self.timings_feedback_update)

        self.yaw = 0.0
        self.b_value = [0.0, 0.0]
        self.prec_yaw = 0.0

    def receive_probabilities(self, msg):
        self.values = msg.softpredict.data

    def receive_yaw(self, msg):
        self.prec_yaw = self.yaw
        self.yaw = msg.angular.z

    def receive_values_bars(self, msg):
        self.b_value = msg.data


    def reset_bci(self):
        rospy.wait_for_service('/integrator/reset')
        resbci = rospy.ServiceProxy('/integrator/reset', Empty)
        try:
            resbci()
            return True
        except rospy.ServiceException as e:
            print("Service call failed: %s")
            return False

    def get_bar_thresholds(self):
        self.threshold_soft_bars = []
        self.threshold_hard_bars = []

        th_soft = rospy.get_param('~thresholds_soft').replace(" ", "").split(',')
        for th in th_soft:
            self.threshold_soft_bars.append(float(th))
            
        th_hard = rospy.get_param('~thresholds_hard').replace(" ", "").split(',')
        for th in th_hard:
            self.threshold_hard_bars.append(float(th))


    def run(self):

        ##### Configure GUI engine #####
        gui = SMRGUI_DWHEEL(rospy.get_param('~window_height'),rospy.get_param('~window_width'),rospy.get_param('~window_scale'))
        gui.init_wheel_bar()
        gui.init_bars()
        
        #print(self.threshold_angles)	
        gui.set_th_left(self.threshold_angles[0])
        gui.set_th_right(self.threshold_angles[1])
        
        gui.set_ith_left(self.threshold_iangles[0])
        gui.set_ith_right(self.threshold_iangles[1])

        #gui.set_bar_thresholds(self.threshold_soft_bars)
        #gui.set_bar_thresholds(self.threshold_hard_bars)
        
        gui.set_yaw(self.yaw)
        gui.draw()

        print("[smrbci] Protocol starts")
        #cv2.waitKey(self.timings_begin)

        exit = False

        ##### Continuous feedback #####
        self.rec_prob = numpy.zeros(self.n_classes)
        self.reset_bci()
        rospy.sleep(0.150)
        publish_neuro_event(self.event_pub, CFEEDBACK)

        while not exit:
            #rospy.spin()
            self.time_checker.make_tic()

            if abs(self.values[0] - self.rec_prob[0]) > 0.00001 or abs(self.yaw - self.prec_yaw) > 0.00001:
                self.rec_prob = self.values

                value = normalize_probabilities_wheel_bar(1.0 - self.rec_prob[0], 1, -1, 1, 0)
                gui.set_value_wheel_bar(value)
                gui.set_yaw(self.yaw)
                gui.set_bar_values(self.b_value)

                
                self.time_checker.make_toc()
                delay = self.time_checker.check_delay()
                if delay < 0:
                    if check_exit(cv2.waitKey(-delay)): exit=True

        publish_neuro_event(self.event_pub, CFEEDBACK+OFF)

        print("[smrbci] Protocol ends")
        cv2.waitKey(self.timings_end)
