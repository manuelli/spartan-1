import numpy as np
import rospy
import time
import std_msgs.msg
import sensor_msgs.msg


diff_threshold = 10


class ExternalForceMonitor(object):

    def __init__(self, diff_threshold):
        self.pub = rospy.Publisher("/stop", std_msgs.msg.Bool, queue_size=10)
        rospy.Subscriber("/joint_states", sensor_msgs.msg.JointState, self.callback)
        self.first_time = True
        self.diff_threshold = diff_threshold

    def callback(self, msg):
        self.external_force = np.array(list(msg.effort))

        if self.first_time:
            self.first_time = False
            self.external_force_ref = self.external_force
        else:
            diff = np.sum(np.square(self.external_force_ref - self.external_force))

            print "ref: ", self.external_force_ref
            print "cur: ", self.external_force, diff

            if diff > self.diff_threshold:
                self.pub.publish(True)
                print "publising"
                time.sleep(5)
                self.pub.publish(False)
                print "stop publising"
                time.sleep(10)


if __name__ == '__main__':

    rospy.init_node('external_force_monitor')
    external_force_monitor = ExternalForceMonitor(diff_threshold)

    rospy.spin()
