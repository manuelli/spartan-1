# system
import os
import sys
import numpy as np
import subprocess
import time

# image processing 
import cv2
from skimage.measure import compare_ssim as ssim

# ros
import rospy
import tf2_ros

# spartan
import spartan.utils.utils as spartanUtils

# touch supervisor
from touch_supervisor import TouchSupervisor

# webcam_monitor
from webcam_monitor import WebcamMonitor


num_scheduled_touch = 1
num_record = 140
scene_sim_threshold = 0.45

touch_space = np.array([[0.45, -0.35, 0.1], [0.82, 0.35, 0.5]])


class TFWrapper(object):

    def __init__(self):
        self.tfBuffer = None
        self.tfListener = None
        self.setup()

    def setup(self):
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

    def getBuffer(self):
        while self.tfBuffer is None:
            time.sleep(0.1)

        return self.tfBuffer


def select_touch_point(idx):

    return np.random.uniform(touch_space[0], touch_space[1])

    '''
    if idx == 0:
        return np.array([0.61, -0.15, 0.5])
    else:
        return np.array([0.55, -0.3, 0.1])
    '''


def start_monitor(name, idx, num_record):
    script_name = os.path.join(spartanUtils.getSpartanSourceDir(), 'yunzhu',
                               'scripts', name + "_monitor.py")
    cmd = "python " + script_name + " " + str(idx) + " " + str(num_record)
    return subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE)


def scene_been_moved(idx):
    dir_path = os.path.join(spartanUtils.getSpartanSourceDir(), 'yunzhu',
                            'data', 'webcam_rec', 'webcam_rec_' + str(idx))
    img_0 = np.array(cv2.imread(dir_path + "/frame_0.jpg")).astype(np.float)
    img_1 = np.array(cv2.imread(dir_path + "/frame_1.jpg")).astype(np.float)

    sim = ssim(img_0, img_1, multichannel=True)

    print "scene similarity:", sim

    if sim < scene_sim_threshold:
        return True
    else:
        return False


def main():
    rospy.init_node('explore_object_node')

    tfWrapper = TFWrapper()
    tfBuffer = tfWrapper.getBuffer()

    touchSupervisor = TouchSupervisor.makeDefault(tfBuffer=tfBuffer)

    for idx in xrange(num_scheduled_touch):

        touchSupervisor.moveHome()

        if idx == 0 or scene_been_moved(idx - 1):
            touchSupervisor.collectSensorDataAndFuse()
            touchSupervisor.moveHome()

        while True:
            touch_point = select_touch_point(idx)
            valid_touch_frame = touchSupervisor.requestTouch(touch_point)
            result = touchSupervisor.waitForGenerateTouchesResult()

            find_touch = touchSupervisor.processGenerateTouchesResult(result)
            if not find_touch:
                continue
            valid_ik = touchSupervisor.solveIK()
            if not valid_ik:
                continue

            print "start webcam monitor"
            webcam_proc = start_monitor('webcam', idx, num_record)
            print "start gelsight monitor"
            gelsight_proc = start_monitor('gelsight', idx, num_record)

            time.sleep(0.5)
            touchSupervisor.attemptTouch()

            break

        touchSupervisor.moveHome()

        gelsight_proc.wait()
        print "gelsight monitor stopped"
        webcam_proc.wait()
        print "webcam monitor stopped"



if __name__ == "__main__":
    main()
