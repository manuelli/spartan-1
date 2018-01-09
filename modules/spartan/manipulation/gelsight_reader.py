import cv2
import numpy as np
import rospy
import std_msgs.msg
import os
import time


threshold = 5

def monitor_gelsight():
    cap = cv2.VideoCapture(2)
    cap.set(3, 640)
    cap.set(4, 480)

    fps = cap.get(cv2.CAP_PROP_FPS)
    print "FPS:", fps

    if(cap.isOpened() == False):
        print "Unable to read camera feed"

    frame_width = int(cap.get(3))
    frame_height = int(cap.get(4))

    print frame_width, frame_height

    rospy.init_node("gelsight_reader_node")
    pub = rospy.Publisher("/stop", std_msgs.msg.Bool, queue_size=10)

    for i in xrange(1000):
        ret, frame = cap.read()
        frame = np.array(frame).astype(np.float)
        print frame.shape

        if i == 0:
            ref = frame
            cv2.imwrite("gelsight_ref.png", ref)
        else:
            diff = np.average(np.abs(ref - frame))
            print "#%d" % i, diff
            if diff > threshold:
                rospy.loginfo("Gelsight in contact #%d" % i)
                pub.publish(True)
                time.sleep(0.01)
                pub.publish(False)


if __name__ == '__main__':
    monitor_gelsight()
