# system
import os
import sys
import numpy as np
import subprocess

# image processing
from skimage.measure import compare_ssim as ssim

# ros
import rospy
import tf2_ros

# spartan
import spartan.utils.utils as spartanUtils

# touch supervisor
from touch_supervisor import TouchSupervisor


num_scheduled_touch = 1


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
    if idx == 0:
        return np.array([0.61, -0.15, 0.5])
    else:
        return np.array([0.55, -0.3, 0.1])


def main():
    rospy.init_node('explore_object_node')

    tfWrapper = TFWrapper()
    tfBuffer = tfWrapper.getBuffer()

    touchSupervisor = TouchSupervisor.makeDefault(tfBuffer=tfBuffer)

    for idx in xrange(num_scheduled_touch):
        touchSupervisor.moveHome()
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

            script_name = os.path.join(spartanUtils.getSpartanSourceDir(), 'yunzhu', 'scripts', 'gelsight_monitor.py')
            cmd = "python " + script_name + " " + str(idx)
            subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE)
            touchSupervisor.attemptTouch()

            break

        touchSupervisor.moveHome()


if __name__ == "__main__":
    main()
