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
        self.threshold = 5
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

        rospy.init_node("gelsight_reader_node")
        self.pub = rospy.Publisher("/stop", std_msgs.msg.Bool, queue_size=10)

        self.sent_signal = False

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

            if i == 0:
                ref = frame
                cv2.imwrite("gelsight_ref.png", ref)
            else:
                diff = np.average(np.abs(ref - frame))
                # print "#%d" % i, diff
                if not self.sent_signal and diff > self.threshold:
                    rospy.loginfo("Gelsight in contact #%d" % i)
                    self.sent_signal = True
                    self.pub.publish(True)
                    time.sleep(0.01)
                    self.pub.publish(False)


if __name__ == '__main__':
    gelsight_monitor = GelsightMonitor(num_record, sys.argv[1])
    gelsight_monitor.start()

