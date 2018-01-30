# system
import sys
import numpy as np
import os
import time
import datetime

# opencv
import cv2

# ROS
import rospy
import std_msgs.msg

# spartan
import spartan.utils.utils as spartanUtils


class WebcamMonitor(object):

    def __init__(self, idx, num_record):
        self.num_record = int(num_record)
        self.idx = idx
        self.cap = cv2.VideoCapture(0)
        self.cap.set(3, 1280)
        self.cap.set(4, 720)

        if(self.cap.isOpened() == False):
            print "Unable to read GelSight feed"

        self.rec_dir_name = os.path.join(spartanUtils.getSpartanSourceDir(),
                                    'yunzhu', 'data', 'webcam_rec',
                                    'webcam_rec_' + self.idx)
        os.system("mkdir -p " + self.rec_dir_name)

    def record(self):
        self.rec_name = os.path.join(self.rec_dir_name, 'webcam_rec_' +
                                     str(datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S-%f")))
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.out = cv2.VideoWriter(self.rec_name + '.avi', fourcc, 20.0, (800, 600))

        for i in xrange(self.num_record):
            ret, frame = self.cap.read()
            frame = frame[69:69+600, 234:234+800, :]

            self.out.write(frame)

            if i == 0:
                cv2.imwrite(self.rec_dir_name + "/frame_0.jpg", frame)
            if i == self.num_record - 1:
                cv2.imwrite(self.rec_dir_name + "/frame_1.jpg", frame)


    def clean(self):
        self.cap.release()
        self.out.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    webcam_monitor = WebcamMonitor(sys.argv[1], sys.argv[2])
    time.sleep(0.8)
    webcam_monitor.record()
    webcam_monitor.clean()
    time.sleep(0.5)

