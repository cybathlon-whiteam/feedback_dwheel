#!/usr/bin/python

from smr_utilities import *

class SmrEvaluationWheel(object):
    def __init__(self):

        ##### Configure publisher #####
        self.event_pub = rospy.Publisher("/events/bus", NeuroEvent, queue_size=1000)

        ##### Configure subscriber #####
        rospy.Subscriber("/integrator/neuroprediction", NeuroOutput, self.receive_probabilities)

        ##### Configure protocol #####
        self.n_classes = rospy.get_param('~n_classes')
        self.n_trials = rospy.get_param('~n_trials')
        str_thr = rospy.get_param('~thresholds') 
        list_thr = str_thr.split(",")
        self.list_thrs = [normalize_th(1.0 - float(list_thr[0]), 1, -1, 1, 0), normalize_th(float(list_thr[1]), 1, -1, 1, 0)]
        self.threshold_angles =  [(self.list_thrs[0] + 1)*180/2, (self.list_thrs[1] + 1)*180/2]
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
        self.timings_timeout = rospy.get_param('~timeout')

        self.time_checker = feedback_timecheck(self.timings_feedback_update)

    def receive_probabilities(self, msg):
        self.values = msg.softpredict.data

    def reset_bci(self):
        rospy.wait_for_service('/integrator/reset')
        resbci = rospy.ServiceProxy('/integrator/reset', Empty)
        try:
            resbci()
            return True
        except rospy.ServiceException as e:
        	print("Service call failed: %s")
        	return False

    def run(self):
        
        ##### Configure protocol #####
        sequence = config_trials(self.n_classes, self.n_trials) 

        ##### Configure GUI engine #####
        gui = SMRGUI_WHEEL(rospy.get_param('~window_height'),rospy.get_param('~window_width'),rospy.get_param('~window_scale'))
        gui.init_wheel_bar()
        gui.set_th_left(self.threshold_angles[0])
        gui.set_th_right(self.threshold_angles[1])
        gui.draw()

        print("[smrbci] Protocol starts")
        cv2.waitKey(self.timings_begin)

        exit = False
        for i,idx in enumerate(sequence):
            print("Trial " + str(i+1) + "/" + str(len(sequence)) + " [" + CLASSES[idx] + "]")
            ##### Fixation #####
            publish_neuro_event(self.event_pub, FIXATION)
            gui.add_fixation()
            if check_exit(cv2.waitKey(self.timings_fixation)): exit=True
            publish_neuro_event(self.event_pub, FIXATION+OFF)
            gui.remove_fixation()

            ##### Cue #####
            publish_neuro_event(self.event_pub,CLASS_EVENTS[idx])
            gui.add_cue(idx)
            if check_exit(cv2.waitKey(self.timings_cue)): exit=True
            publish_neuro_event(self.event_pub, CLASS_EVENTS[idx]+OFF)

            ##### Continuous feedback #####
            self.rec_prob = numpy.zeros(self.n_classes)
            hit = False
            self.reset_bci()
            rospy.sleep(0.150)
            publish_neuro_event(self.event_pub, CFEEDBACK)
            timeout_reached = False
            start_time = time.time()

            while (not hit) and (not timeout_reached):
                self.time_checker.make_tic()

                #rospy.spin()
                if abs(self.values[0] - self.rec_prob[0]) > 0.00001:
                    self.rec_prob = self.values

                    if idx == 3:
                        value = 0 # in this way with idx 3 the move bar is fixed at 90 degree
                    else:
                        value = normalize_probabilities_wheel_bar(1.0 - self.rec_prob[0], 1, -1, 1, 0)
                    gui.set_value_wheel_bar(value)

                    ths = self.list_thrs

                    # we have continuous control
                    if value >= ths[1]:
                        hit = True
                        c = 1
                        color = (50,205,50)
                    elif value <= ths[0]:
                        hit = True
                        c = 0
                        color = (128,0,128)
                
                self.time_checker.make_toc()
                delay = self.time_checker.check_delay()
                if delay < 0:
                    if check_exit(cv2.waitKey(-delay)): exit=True

                c_time = time.time()
                if ((c_time-start_time) >= self.timings_timeout):
                    timeout_reached = True
                    color = (100,100,100)
                    c = -1

            publish_neuro_event(self.event_pub, CFEEDBACK+OFF)

            ##### Boom #####
            gui.set_color_wheel_bar(color)
            cv2.waitKey(100)
            if c == -1:
                publish_neuro_event(self.event_pub, TIMEOUT)
                if check_exit(cv2.waitKey(self.timings_boom)): exit=True
                publish_neuro_event(self.event_pub, TIMEOUT+OFF)
            elif c == idx:
                publish_neuro_event(self.event_pub, TARGETHIT)
                if check_exit(cv2.waitKey(self.timings_boom)): exit=True
                publish_neuro_event(self.event_pub, TARGETHIT+OFF)
            else:
                publish_neuro_event(self.event_pub, TARGETMISS)
                if check_exit(cv2.waitKey(self.timings_boom)): exit=True
                publish_neuro_event(self.event_pub, TARGETMISS+OFF)
            gui.reset_wheel_bar()
            gui.remove_cue()

            if check_exit(cv2.waitKey(self.timings_iti)): exit=True
            if exit:
                print("User asked to quit")
                break

        print("[smrbci] Protocol ends")
        cv2.waitKey(self.timings_end)
