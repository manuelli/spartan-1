# system
import os
import sys
import numpy as np
import subprocess
import time
import datetime

# image processing 
import cv2

# ros
import rospy
import tf2_ros
import rosbag

# spartan
import spartan.utils.utils as spartanUtils

# touch supervisor
from touch_supervisor import TouchSupervisor

# webcam_monitor
from webcam_monitor import WebcamMonitor


num_scheduled_touch = 50
record_time = 11
scene_sim_threshold = 0.45
rescan_cycle = 50

touch_space = np.array([[0.50, -0.33, 0.03], [0.78, 0.33, 0.2]])


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


def start_streamer(name, idx, record_time, src, height, width, fps):
    cmd = 'streamer -c ' + str(src)
    cmd += ' -t 0:' + str(record_time)
    cmd += ' -s ' + str(width) + "x" + str(height)
    cmd += ' -r ' + str(fps)
    cmd += ' -f jpeg'

    dir_name = os.path.join(spartanUtils.getSpartanSourceDir(), 'yunzhu',
                            'data', name + '_rec')
    os.system("mkdir -p " + dir_name)

    rec_name = os.path.join(dir_name, 'webcam_rec_' + str(idx) + '_' +
                            str(datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S-%f")) + '.avi')

    cmd += ' -o ' + rec_name

    return subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE)


'''
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
'''


def store_point_cloud_list_msg(idx, touchSupervisor):
    pointCloudListMsg = touchSupervisor.pointCloudListMsg
    dir_path = os.path.join(spartanUtils.getSpartanSourceDir(), 'yunzhu', 'data',
                            'point_cloud_rec')
    os.system('mkdir -p ' + dir_path)

    bag = rosbag.Bag(os.path.join(dir_path, 'point_cloud_rec_' +
                                  str(datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S-%f")) + '.bag'),
                     'w')
    bag.write('pointCloudListMsg', pointCloudListMsg)
    bag.close()


def process_results_to_touch_soft_object(result):
    if len(result.scored_touches) == 0:
        return result
    pose = result.scored_touches[0].pose.pose
    params = result.scored_touches[0].params

    touchFrameXAxis = spartanUtils.transformFromROSPoseMsg(pose).TransformVector(1, 0, 0)
    pose.position.x += touchFrameXAxis[0] * params.finger_depth
    pose.position.y += touchFrameXAxis[1] * params.finger_depth

    pose.position.z = 0.05
    pose.orientation.x = 0
    pose.orientation.y = 0.7071068
    pose.orientation.z = 0
    pose.orientation.w = 0.7071068

    result.scored_touches[0].pose.pose = pose
    return result


def main():
    rospy.init_node('explore_object_node')

    tfWrapper = TFWrapper()
    tfBuffer = tfWrapper.getBuffer()

    touchSupervisor = TouchSupervisor.makeDefault(tfBuffer=tfBuffer)

    start_idx = int(sys.argv[1])
    soft_object = bool(int(sys.argv[2]))

    for idx in xrange(start_idx, start_idx + num_scheduled_touch):

        if idx == start_idx or (idx - start_idx) % rescan_cycle == 0:
            touchSupervisor.moveHome()
            touchSupervisor.collectSensorDataAndFuse()
            store_point_cloud_list_msg(idx, touchSupervisor)

        touchSupervisor.moveTouchReady()

        while True:
            touch_point = select_touch_point(idx)
            valid_touch_frame = touchSupervisor.requestTouch(touch_point)
            result = touchSupervisor.waitForGenerateTouchesResult()

            if soft_object:
                result = process_results_to_touch_soft_object(result)

            find_touch = touchSupervisor.processGenerateTouchesResult(result)
            if not find_touch:
                continue
            valid_ik = touchSupervisor.solveIK()
            if not valid_ik:
                continue

            print "start pose monitor"
            pose_proc = start_monitor('pose', idx, record_time * 110)
            time.sleep(3)

            print "start webcam monitor"
            webcam_proc = start_streamer('webcam', idx, record_time, '/dev/video1', 720, 1280, 24)
            print "start gelsight monitor"
            gelsight_proc = start_streamer('gelsight', idx, record_time, '/dev/video0', 720, 960, 24)

            time.sleep(0.5)

            touchSupervisor.attemptTouch()

            break

        touchSupervisor.moveTouchReady()

        gelsight_proc.wait()
        print "gelsight monitor stopped"
        webcam_proc.wait()
        print "webcam monitor stopped"
        pose_proc.wait()
        print 'pose monitor stopped'



if __name__ == "__main__":
    main()
