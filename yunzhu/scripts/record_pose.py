import os
import sys
import numpy as np
import tf2_ros
import rospy


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


def main():

    rospy.init_node('record_pose_node');

    filename = str(sys.argv[1])
    fout = open(filename, 'w')

    tfWrapper = TFWrapper()
    tfBuffer = tfWrapper.getBuffer()

    touchFrameName = 'base'
    rgbOpticalFrameName = 'camera_1112170110_rgb_optical_frame'

    rospy.sleep(0.5)

    while(1):
        transform = getRgbOpticalFrameToTouchFrameTransform(tfBuffer, touchFrameName, rgbOpticalFrameName)
        fout.write(str(transform.header.stamp.secs) + str(transform.header.stamp.nsecs) + ",")
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        fout.write(str(translation.x) + "," + str(translation.y) + "," + str(translation.z) + ",")
        fout.write(str(rotation.x) + "," + str(rotation.y) + "," + str(rotation.z) + "," + str(rotation.w) + "\n")
        rospy.sleep(0.005)



if __name__ == '__main__':
    main()
