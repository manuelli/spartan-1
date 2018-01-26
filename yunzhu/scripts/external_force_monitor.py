import numpy as np
import rospy
import time
import std_msgs.msg
import sensor_msgs.msg


diff_threshold = 6


class ExternalForceMonitor(object):

    def __init__(self, diff_threshold):
        self.pub = rospy.Publisher("/stop", std_msgs.msg.Bool, queue_size=10)
        rospy.Subscriber("/stop", std_msgs.msg.Bool, self.encounter_stop_signal)
        rospy.Subscriber("/joint_states", sensor_msgs.msg.JointState, self.callback)
        self.first_time = True
        self.diff_threshold = diff_threshold
        self.sent_signal = False

    def encounter_stop_signal(self, msg):
        if msg.data:
            self.sent_signal = True

    def callback(self, msg):
        self.external_force = np.array(list(msg.effort))

        if self.first_time:
            self.first_time = False
            self.external_force_ref = self.external_force
        else:
            diff = np.sum(np.square(self.external_force_ref - self.external_force))

            # print "ref: ", self.external_force_ref
            # print "cur: ", self.external_force, diff

            if not self.sent_signal and diff > self.diff_threshold:
                self.sent_signal = True
                self.pub.publish(True)
                print "publising"
                time.sleep(0.01)
                self.pub.publish(False)
                print "stop publising"


if __name__ == '__main__':

    time.sleep(1.0)

    rospy.init_node('external_force_monitor')
    external_force_monitor = ExternalForceMonitor(diff_threshold)

    rospy.spin()
