# system
import sys
import numpy as np
import os
import time

# opencv
import cv2

# ROS
import rospy
import std_msgs.msg

# spartan
import spartan.utils.utils as spartanUtils


num_record = 600

class GelsightMonitor(object):

    def __init__(self, num_record, idx):
        self.num_record = num_record
        self.idx = idx
        self.cap = cv2.VideoCapture(0)
        self.cap.set(3, 640)
        self.cap.set(4, 480)
        self.fps = self.cap.get(cv2.CAP_PROP_FPS)
        print "FPS:", self.fps

        if(self.cap.isOpened() == False):
            print "Unable to read camera feed"

        self.frame_width = int(self.cap.get(3))
        self.frame_height = int(self.cap.get(4))

        print self.frame_width, self.frame_height


    def start(self):

        rec_dir_name = os.path.join(spartanUtils.getSpartanSourceDir(),
                                    'yunzhu', 'data', 'gelsight_rec',
                                    'gelsight_rec_' + self.idx)
        os.system("mkdir -p " + rec_dir_name)

        for i in xrange(self.num_record):
            ret, frame = self.cap.read()
            frame = np.array(frame).astype(np.float)
            # print frame.shape

            cv2.imwrite(rec_dir_name + "/frame_" + str(i) + ".jpg", frame)


if __name__ == '__main__':
    gelsight_monitor = GelsightMonitor(num_record, sys.argv[1])
    gelsight_monitor.start()

