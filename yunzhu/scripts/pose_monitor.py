# system
import os
import sys
import numpy as np
import time
import datetime

# ros
import tf2_ros
import rospy

# spartan
import spartan.utils.utils as spartanUtils


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


def getRgbOpticalFrameToTouchFrameTransform(tfBuffer, touchFrameName, rgbOpticalFrameName):
    rgbOpticalFrameToTouchFrame = tfBuffer.lookup_transform(touchFrameName, rgbOpticalFrameName, rospy.Time(0))
    return rgbOpticalFrameToTouchFrame


def main(idx, num_record):

    # ros operation
    rospy.init_node('record_pose_node');

    tfWrapper = TFWrapper()
    tfBuffer = tfWrapper.getBuffer()

    touchFrameName = 'base'
    rgbOpticalFrameName = 'camera_1112170110_rgb_optical_frame'

    # file operation
    rec_dir_name = os.path.join(spartanUtils.getSpartanSourceDir(), 'yunzhu', 'data', 'pose_rec')
    os.system("mkdir -p " + rec_dir_name)
    rec_name = os.path.join(rec_dir_name, 'pose_rec_' + idx)
    fout = open(rec_name, "w")

    rospy.sleep(0.5)

    for i in xrange(int(num_record)):
        transform = getRgbOpticalFrameToTouchFrameTransform(tfBuffer, touchFrameName, rgbOpticalFrameName)
        fout.write(str(datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S-%f")) + " ")
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        fout.write(str(translation.x) + "," + str(translation.y) + "," + str(translation.z) + ",")
        fout.write(str(rotation.x) + "," + str(rotation.y) + "," + str(rotation.z) + "," + str(rotation.w) + "\n")
        rospy.sleep(0.01)

    fout.close()


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2])
