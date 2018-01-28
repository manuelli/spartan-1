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


class GelsightMonitor(object):

    def __init__(self, idx, num_record):
        self.num_record = int(num_record)
        self.idx = idx
        self.cap = cv2.VideoCapture(0)
        self.cap.set(3, 960)
        self.cap.set(4, 720)

        if(self.cap.isOpened() == False):
            print "Unable to read GelSight feed"

        rec_dir_name = os.path.join(spartanUtils.getSpartanSourceDir(),
                                    'yunzhu', 'data', 'gelsight_rec',
                                    'gelsight_rec_' + self.idx)
        os.system("mkdir -p " + rec_dir_name)

        rec_name = os.path.join(rec_dir_name, 'gelsight_rec_' +
                                str(datetime.datetime.now()))
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.out = cv2.VideoWriter(rec_name + '.avi', fourcc, 20.0, (960, 720))


    def record(self):
        for i in xrange(self.num_record):
            ret, frame = self.cap.read()
            self.out.write(frame)


    def clean(self):
        self.cap.release()
        self.out.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    gelsight_monitor = GelsightMonitor(sys.argv[1], sys.argv[2])
    time.sleep(1)
    gelsight_monitor.record()
    gelsight_monitor.clean()
    time.sleep(0.5)

