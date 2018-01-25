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


num_record = 500

class GelsightMonitor(object):

    def __init__(self, num_record, idx):
        self.num_record = num_record
        self.threshold = 4
        self.idx = idx
        self.cap = cv2.VideoCapture(0)
        self.cap.set(3, 640)
        self.cap.set(4, 480)
        self.fps = cap.get(cv2.CAP_PROP_FPS)
        print "FPS:", fps

        if(cap.isOpened() == False):
            print "Unable to read camera feed"

        self.frame_width = int(cap.get(3))
        self.frame_height = int(cap.get(4))

        print frame_width, frame_height

        rospy.init_node("gelsight_reader_node")
        pub = rospy.Publisher("/stop", std_msgs.msg.Bool, queue_size=10)

    def start(self):

        rec_dir_name = os.path.join(spartanUtils.getSpartanSourceDir(),
                                    'yunzhu', 'data', 'gelsight_rec',
                                    'gelsight_rec' + self.idx)
        os.system("mkdir -p " + rec_dir_name)

        flag = False
        for i in xrange(self.num_record):
            ret, frame = cap.read()
            frame = np.array(frame).astype(np.float)
            print frame.shape

            cv2.imwrite(rec_dir_name + "/frame_" + str(i) + ".jpg", frame)

            if i == 0:
                ref = frame
                cv2.imwrite("gelsight_ref.png", ref)
            else:
                diff = np.average(np.abs(ref - frame))
                print "#%d" % i, diff
                if diff > threshold:
                    rospy.loginfo("Gelsight in contact #%d" % i)
                    if flag == False:
                        flag = True
                        pub.publish(True)
                        time.sleep(0.01)
                        pub.publish(False)
                else:
                    flag = False


if __name__ == '__main__':
    gelsight_monitor = GelsightMonitor(num_record, sys.argv[1])
    gelsight_monitor.start()

