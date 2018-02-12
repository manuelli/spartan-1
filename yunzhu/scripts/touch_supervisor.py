# system
import os
import numpy as np

# ROS
import rospy
import sensor_msgs.msg
import geometry_msgs.msg
import std_msgs.msg
import tf2_ros
import rosbag
import subprocess
import actionlib
from tf.transformations import quaternion_from_euler
from fusion_server.srv import *
from numpy_pc2 import array_to_xyz_pointcloud2f

# spartan ROS
import spartan_touch_msgs.msg
import spartan_touch_msgs.srv

# spartan
import spartan.utils.utils as spartanUtils
import spartan.utils.ros_utils as spartanROSUtils

# director
from director import transformUtils

# LCM
import lcm
from robotlocomotion import robot_plan_t

# ply reader
from plyfile import PlyData, PlyElement


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


class TouchSupervisor(object):

    def __init__(self, touchParamsFile=None, cameraSerialNumber='carmine_1', tfBuffer=None):
        self.touchParamsFile = touchParamsFile
        self.reloadParams()
        self.cameraSerialNumber = cameraSerialNumber

        self.cameraName = 'camera_' + str(cameraSerialNumber)
        self.pointCloudTopic = '/' + str(self.cameraName) + '/depth/points'
        self.touchFrameName = 'base'
        self.depthOpticalFrameName = self.cameraName + "_depth_optical_frame"
        self.rgbOpticalFrameName = self.cameraName + "_rgb_optical_frame"

        self.robotService = spartanROSUtils.RobotService.makeKukaRobotService()
        self.tfBuffer = tfBuffer
        self.setup()

        # subscribe to ros topic for stopping
        rospy.Subscriber("/joint_states", sensor_msgs.msg.JointState, self.storeCurrentJointPosition)
        rospy.Subscriber("/stop", std_msgs.msg.Bool, self.stopCurrentPoint)


    '''
    setup
    '''
    def reloadParams(self):
        self.touchParams = spartanUtils.getDictFromYamlFilename(self.touchParamsFile)

    def setup(self):
        self.setupConfig()
        self.setupSubscribers()
        self.setupTF()
        self.setupROSActions()

    def setupConfig(self):
        params = self.touchParams
        self.touchToIiwaLinkEE = spartanUtils.transformFromPose(params['touch']['touch_to_ee'])
        self.iiwaLinkEEToTouchFrame = self.touchToIiwaLinkEE.GetLinearInverse()

        pos = [-0.10, 0, 0]
        quat = [1, 0, 0, 0]
        self.preTouchToTouchTransform = transformUtils.transformFromPose(pos, quat)


    def setupSubscribers(self):
        self.pointCloudSubscriber = spartanROSUtils.SimpleSubscriber(self.pointCloudTopic, sensor_msgs.msg.PointCloud2)
        self.pointCloudSubscriber.start()

    def setupROSActions(self):
        actionName = '/spartan_touch/GenerateTouchesFromPointCloudList'
        self.generate_touches_client = actionlib.SimpleActionClient(actionName, spartan_touch_msgs.msg.GenerateTouchesFromPointCloudListAction)

    def setupTF(self):
        if self.tfBuffer is None:
            self.tfBuffer = tf2_ros.Buffer()


    '''
    stop upon signal
    '''
    def storeCurrentJointPosition(self, msg):
        self.currentJointPosition = msg.position

    def stopCurrentPoint(self, msg):
        if msg.data:
            rospy.loginfo("stopping robot")
            lcm_msg = robot_plan_t()
            lcm_msg.utime = 0
            lc = lcm.LCM()
            lc.publish("STOP", lcm_msg.encode())


    '''
    utils
    cartesian_point is a list of 3 floats for x, y, and z (ex: [0.1, 0.2, 0.3])
    This format can be changed to reflect how the points are estimated from the point cloud.
    '''
    def getJointPositions(self, pose_):
        pose = geometry_msgs.msg.Pose()
        pose.position.x = pose_[0]
        pose.position.y = pose_[1]
        pose.position.z = pose_[2]

        pose.orientation.w = pose_[3]
        pose.orientation.x = pose_[4]
        pose.orientation.y = pose_[5]
        pose.orientation.z = pose_[6]

        poseStamped = geometry_msgs.msg.PoseStamped()
        poseStamped.pose = pose
        poseStamped.header.frame_id = "base"

        response = self.robotService.runIK(poseStamped)
        if not response.success:
            rospy.loginfo("ik was not successful, returning without moving robot")
            return

        rospy.loginfo("ik was successful, returning joint position")
        return response.joint_state.position

    def getIiwaLinkEEFrameFromTouchFrame(self, touchFrame):
        return transformUtils.concatenateTransforms([self.iiwaLinkEEToTouchFrame, touchFrame])

    def makePoseStampedFromTouchFrame(self, touchFrame):
        iiwaLinkEEFrame = self.getIiwaLinkEEFrameFromTouchFrame(touchFrame)
        poseDict = spartanUtils.poseFromTransform(iiwaLinkEEFrame)
        poseMsg = spartanROSUtils.ROSPoseMsgFromPose(poseDict)
        poseStamped = geometry_msgs.msg.PoseStamped()
        poseStamped.pose = poseMsg
        poseStamped.header.frame_id = "base"
        return poseStamped

    @staticmethod
    def rectangleMessageFromYamlNode(node):
        msg = spartan_touch_msgs.msg.Rectangle()
        msg.min_pt = spartanROSUtils.listToPointMsg(node['min_pt'])
        msg.max_pt = spartanROSUtils.listToPointMsg(node['max_pt'])
        msg.pose = spartanROSUtils.ROSPoseMsgFromPose(node)
        return msg


    '''
    collect point cloud
    '''
    def getDepthOpticalFrameToTouchFrameTransform(self):
        depthOpticalFrameToTouchFrame = self.tfBuffer.lookup_transform(self.touchFrameName, self.depthOpticalFrameName, rospy.Time(0))
        print depthOpticalFrameToTouchFrame
        return depthOpticalFrameToTouchFrame

    def capturePointCloudAndCameraTransform(self, cameraOrigin=[0, 0, 0]):
        rospy.sleep(0.5)
        msg = spartan_touch_msgs.msg.PointCloudWithTransform()
        msg.header.stamp = rospy.Time.now()

        msg.camera_origin.x = cameraOrigin[0]
        msg.camera_origin.y = cameraOrigin[1]
        msg.camera_origin.z = cameraOrigin[2]

        msg.point_cloud_to_base_transform = self.getDepthOpticalFrameToTouchFrameTransform()
        msg.point_cloud = self.pointCloudSubscriber.waitForNextMessage()

        self.testData = msg
        return msg

    def saveSensorDataToBagFile(self, pointCloudListMsg=None, filename=None, overwrite=True):
        if pointCloudListMsg is None:
            pointCloudListMsg = self.pointCloudListMsg

        if filename is None:
            filename = os.path.join(spartanUtils.getSpartanSourceDir(), 'sandbox', 'touch_sensor_data.bag')

        if overwrite and os.path.isfile(filename):
            os.remove(filename)

        print "Saving sensor data to " + filename
        bag = rosbag.Bag(filename, 'w')
        bag.write('data', pointCloudListMsg)
        bag.close()

    def collectSensorData(self, saveToBagFile=False, **kwargs):
        rospy.loginfo("collecting sensor data")

        pointCloudListMsg = spartan_touch_msgs.msg.PointCloudList()
        pointCloudListMsg.header.stamp = rospy.Time.now()

        data = dict()

        for poseName in self.touchParams['scan_pose_list']:
            rospy.loginfo("moving to pose = " + poseName)
            joint_positions = self.touchParams['poses'][poseName]
            self.robotService.moveToJointPosition(joint_positions, maxJointDegreesPerSecond=self.touchParams['speed']['scan'])

            pointCloudWithTransformMsg = self.capturePointCloudAndCameraTransform()
            pointCloudListMsg.point_cloud_list.append(pointCloudWithTransformMsg)
            data[poseName] = pointCloudWithTransformMsg

        self.sensorData = data
        self.pointCloudListMsg = pointCloudListMsg

        if saveToBagFile:
            self.saveSensorDataToBagFile(pointCloudListMsg, **kwargs)

        # print pointCloudListMsg

        return pointCloudListMsg


    '''
    Elastic Fusion
    '''
    def convert_ply_to_pointcloud2(self, plydata):

        cloud_arr = np.zeros((len(plydata.elements[0].data), 3))
        for i in xrange(len(plydata.elements[0].data)):
            cloud_arr[i] = list(plydata.elements[0].data[i])[:3]

        return array_to_xyz_pointcloud2f(cloud_arr)

    def getRgbOpticalFrameToTouchFrameTransform(self):
        rgbOpticalFrameToTouchFrame = self.tfBuffer.lookup_transform(self.touchFrameName, self.rgbOpticalFrameName, rospy.Time(0))
        print rgbOpticalFrameToTouchFrame
        return rgbOpticalFrameToTouchFrame

    def captureCameraTransform(self, cameraOrigin=[0, 0, 0]):

        rospy.sleep(0.5)

        msg = spartan_touch_msgs.msg.PointCloudWithTransform()
        msg.header.stamp = rospy.Time.now()

        msg.camera_origin.x = cameraOrigin[0]
        msg.camera_origin.y = cameraOrigin[1]
        msg.camera_origin.z = cameraOrigin[2]

        msg.point_cloud_to_base_transform = self.getRgbOpticalFrameToTouchFrameTransform()

        # hack to get the correct pose
        # msg.point_cloud_to_base_transform.transform.translation.x += 0.06
        # msg.point_cloud_to_base_transform.transform.translation.y += 0.055

        return msg

    def collectSensorDataAndFuse(self):
        pointCloudListMsg = spartan_touch_msgs.msg.PointCloudList()
        pointCloudListMsg.header.stamp = rospy.Time.now()

        pointCloudWithTransformMsg = self.captureCameraTransform()

        # capture scene and do elastic fusion
        rospy.loginfo("Waiting for 'capture_scene_and_fuse' service...")
        rospy.wait_for_service('capture_scene_and_fuse')
        rospy.loginfo("Found it!, starting capture...")

        try:
            capture_scene_and_fuse = rospy.ServiceProxy('capture_scene_and_fuse', CaptureSceneAndFuse)
            resp1 = capture_scene_and_fuse()
            bagFilename = resp1.pointcloud_filepath
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

        pointCloudWithTransformMsg.point_cloud = self.convert_ply_to_pointcloud2(PlyData.read(bagFilename + '.ply'))
        os.system('rm ' + bagFilename)
        pointCloudListMsg.point_cloud_list.append(pointCloudWithTransformMsg)

        self.pointCloudListMsg = pointCloudListMsg

        return pointCloudListMsg


    '''
    touch related functions
    '''
    def requestTouch(self, touch_point):
        # request the touch via a ROS Action
        rospy.loginfo("waiting for spartan touch server")
        self.generate_touches_client.wait_for_server()
        rospy.loginfo("requesting touches from spartan touch server")

        params = self.touchParams
        goal = spartan_touch_msgs.msg.GenerateTouchesFromPointCloudListGoal()
        goal.point_clouds = self.pointCloudListMsg

        # predefine the touch point for now
        goal.touch_point.x = touch_point[0]
        goal.touch_point.y = touch_point[1]
        goal.touch_point.z = touch_point[2]

        if 'touch_volume' in params:
            node = params['touch_volume']
            rectangle = TouchSupervisor.rectangleMessageFromYamlNode(node)
            goal.params.touch_volume.append(rectangle)

        if 'collision_volume' in params:
            node = params['collision_volume']
            rectangle = TouchSupervisor.rectangleMessageFromYamlNode(node)
            goal.params.collision_volume.append(rectangle)

        if 'collision_objects' in params:
            for key, val in params['collision_objects'].iteritems():
                rectangle = TouchSupervisor.rectangleMessageFromYamlNode(val)
                goal.params.collision_objects.append(rectangle)

        self.generate_touches_client.send_goal(goal)

    def waitForGenerateTouchesResult(self):
        rospy.loginfo("waiting for result")
        self.generate_touches_client.wait_for_result()
        result = self.generate_touches_client.get_result()
        self.generate_touches_result = result
        rospy.loginfo("received result")

        return result

    def touchFrameXAxisInThreshold(self, touchFrame):
        touchFrameXAxis = touchFrame.TransformVector(1, 0, 0)
        params = self.touchParams
        touchNominalDirectionX = params['touch']['touch_nominal_direction_x']
        proj = np.dot(touchFrameXAxis, touchNominalDirectionX)
        print "Projection on -z axis:", proj
        if proj < 0.93:
            return False
        return True

    def processGenerateTouchesResult(self, result):
        print "num scored_touches = ", len(result.scored_touches)
        if len(result.scored_touches) == 0:
            rospy.loginfo("no valid touches found")
            return False

        self.topTouch = result.scored_touches[0]
        rospy.loginfo("-------- top touch score = %.3f", self.topTouch.score)
        self.touchFrame = spartanUtils.transformFromROSPoseMsg(self.topTouch.pose.pose)
        self.rotateTouchFrameToAlignWithNominal(self.touchFrame)
        return self.touchFrameXAxisInThreshold(self.touchFrame)

    def rotateTouchFrameToAlignWithNominal(self, touchFrame):
        touchFrameZAxis = touchFrame.TransformVector(0, 0, 1)
        params = self.touchParams
        touchNominalDirectionZ = params['touch']['touch_nominal_direction_z']
        if (np.dot(touchFrameZAxis, touchNominalDirectionZ) < 0):
            touchFrame.PreMultiply()
            touchFrame.RotateX(180)

    def start_external_force_monitor(self):
        print "Start external force monitor"
        monitor_script_name = os.path.join(spartanUtils.getSpartanSourceDir(), 'yunzhu', 'scripts', 'external_force_monitor.py')
        cmd = "python " + monitor_script_name
        self.external_force_monitor_proc = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE)

    def stop_external_force_monitor(self, s='/external_force_monitor'):
        print "End external force monitor"
        list_cmd = subprocess.Popen("rosnode list", shell=True, stdout=subprocess.PIPE)
        list_output = list_cmd.stdout.read()
        retcode = list_cmd.wait()
        assert retcode == 0, "List command returned %d" % retcode
        for str in list_output.split("\n"):
            if (str.startswith(s)):
                os.system("rosnode kill " + str)

    def solveIK(self):
        preTouchFrame = transformUtils.concatenateTransforms([self.preTouchToTouchTransform, self.touchFrame])

        params = self.touchParams
        above_table_pre_touch = params['poses']['above_table_pre_touch']
        preTouchFramePoseStamped = self.makePoseStampedFromTouchFrame(preTouchFrame)
        preTouch_ik_response = self.robotService.runIK(preTouchFramePoseStamped, seedPose=above_table_pre_touch, nominalPose=above_table_pre_touch)

        if not preTouch_ik_response.success:
            rospy.loginfo("pre touch pose ik failed, returning")
            return False

        touchFramePoseStamped = self.makePoseStampedFromTouchFrame(self.touchFrame)
        self.preTouchPose = preTouch_ik_response.joint_state.position

        touch_ik_response = self.robotService.runIK(touchFramePoseStamped,
                                                    seedPose=self.preTouchPose,
                                                    nominalPose=self.preTouchPose)

        if not touch_ik_response.success:
            rospy.loginfo("touch pose not reachable, returning")
            return False

        self.touchPose = touch_ik_response.joint_state.position

        return True

    def attemptTouch(self):
        params = self.touchParams
        self.robotService.moveToJointPosition(self.preTouchPose, maxJointDegreesPerSecond=params['speed']['pre_touch'])

        self.start_external_force_monitor()
        self.robotService.moveToJointPosition(self.touchPose, maxJointDegreesPerSecond=params['speed']['touch'])
        self.stop_external_force_monitor()

        return True

    '''
    basic functions
    '''
    def moveHome(self):
        rospy.loginfo("moving home")
        homePose = self.touchParams['poses']['scan_back']
        self.robotService.moveToJointPosition(homePose, maxJointDegreesPerSecond=self.touchParams['speed']['nominal'])

    def moveTouchReady(self):
        rospy.loginfo("moving touch ready")
        readyPose = self.touchParams['poses']['above_table_pre_touch']
        self.robotService.moveToJointPosition(readyPose, maxJointDegreesPerSecond=self.touchParams['speed']['nominal'])

    def exploreObject(self, homePose, touchPoses):
        homeJointPosition = self.getJointPositions(homePose)
        touchJointPositions = [self.getJointPositions(pose) for pose in touchPoses]
        self.robotService.moveToJointPosition(homeJointPosition, self.touchParams['speed']['nominal'])

        for point in touchJointPositions:
            self.robotService.moveToJointPosition(point, self.touchParams['speed']['nominal'])
            self.robotService.moveToJointPosition(homeJointPosition, self.touchParams['speed']['nominal'])


    @staticmethod
    def makeDefault(**kwargs):
        touchParamsFile = os.path.join(spartanUtils.getSpartanSourceDir(),
                                       'yunzhu', 'config', 'params_touch.yaml')
        return TouchSupervisor(touchParamsFile=touchParamsFile, **kwargs)


    '''
    test functions
    '''
    def testExploreObject(self):
        homePose = [0.49343, 0.05433, 0.78004, 0.707, 0., 0.707, 0.]
        poseA = [0.451322943411, 0.487866748385, 0.5614778903, 0.707, 0., 0.707, 0.]
        poseB = [0.495035021527, -0.504178258792, 0.578621340566, 0.707, 0., 0.707, 0.]
        touchPoses = [poseA, poseB]
        self.exploreObject(homePose, touchPoses)
        self.moveHome()

    def testCollectSensorData(self):
        self.collectSensorData()

    def testCollectSensorDataAndGenerateTouches(self, touch_points):
        for touch_point in touch_points:
            self.moveHome()
            self.collectSensorDataAndFuse()
            # self.collectSensorData()
            self.requestTouch(touch_point)
            self.moveHome()
            result = self.waitForGenerateTouchesResult()
            self.processGenerateTouchesResult(result)

    def testAttemptTouch(self):
        self.attemptTouch()

    def testPlanAndTouchObject(self, touch_points):
        for touch_point in touch_points:
            # self.collectSensorData()
            self.collectSensorDataAndFuse()
            self.requestTouch(touch_point)
            self.moveHome()
            result = self.waitForGenerateTouchesResult()
            self.processGenerateTouchesResult(result)
            self.attemptTouch()
            self.moveHome()


def main():
    rospy.init_node('touch_supervisor_node')

    tfWrapper = TFWrapper()
    tfBuffer = tfWrapper.getBuffer()

    touch_point_0 = np.array([0.61, -0.15, 0.5])
    # touch_point_1 = np.array([0.7, -0.2, 0.3])
    touch_points = [touch_point_0] #, touch_point_1]

    touchSupervisor = TouchSupervisor.makeDefault(tfBuffer=tfBuffer)
    touchSupervisor.testCollectSensorDataAndGenerateTouches(touch_points)
    touchSupervisor.testAttemptTouch()
    # touchSupervisor.testPlanAndTouchObject(touch_points)


if __name__ == "__main__":
    main()
