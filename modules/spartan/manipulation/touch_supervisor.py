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
import actionlib
from tf.transformations import quaternion_from_euler

# spartan ROS
import spartan_touch_msgs.msg
import spartan_touch_msgs.srv

#spartan
import spartan.utils.utils as spartanUtils
import spartan.utils.ros_utils as spartanROSUtils

# director
from director import transformUtils

# LCM
import lcm
from robotlocomotion import robot_plan_t


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

    def __init__(self, touchParamsFile=None, cameraSerialNumber=1112170110, tfBuffer=None):
        self.touchParamsFile = touchParamsFile
        self.reloadParams()
        self.cameraSerialNumber = cameraSerialNumber

        self.cameraName = 'camera_' + str(cameraSerialNumber)
        self.pointCloudTopic = '/' + str(self.cameraName) + '/depth/points'
        self.touchFrameName = 'base'
        self.depthOpticalFrameName = self.cameraName + "_depth_optical_frame"

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
    touch related functions
    '''
    def requestTouch(self):
        # request the touch via a ROS Action
        rospy.loginfo("waiting for spartan touch server")
        self.generate_touches_client.wait_for_server()
        rospy.loginfo("requesting touches from spartan touch server")

        params = self.touchParams
        goal = spartan_touch_msgs.msg.GenerateTouchesFromPointCloudListGoal()
        goal.point_clouds = self.pointCloudListMsg

        # predefine the touch point for now
        goal.touch_point.touch_point.x = 0.6
        goal.touch_point.touch_point.y = 0.0
        goal.touch_point.touch_point.z = 0.5

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

    def processGenerateTouchesResult(self, result):
        print "num scored_touches = ", len(result.scored_touches)
        if len(result.scored_touches) == 0:
            rospy.loginfo("no valid touches found")
            return false

        self.topTouch = result.scored_touches[0]
        rospy.loginfo("-------- top touch score = %.3f", self.topTouch.score)
        self.touchFrame = spartanUtils.transformFromROSPoseMsg(self.topTouch.pose.pose)
        self.rotateTouchFrameToAlignWithNominal(self.touchFrame)
        return True

    def rotateTouchFrameToAlignWithNominal(self, touchFrame):
        touchFrameZAxis = touchFrame.TransformVector(0, 0, 1)
        params = self.touchParams
        touchNominalDirection = params['touch']['touch_nominal_direction']
        if (np.dot(touchFrameZAxis, touchNominalDirection) < 0):
            touchFrame.PreMultiply()
            touchFrame.RotateX(180)

    def attemptTouch(self, touchFrame):
        preTouchFrame = transformUtils.concatenateTransforms([self.preTouchToTouchTransform, self.touchFrame])

        params = self.touchParams
        above_table_pre_touch = params['poses']['above_table_pre_touch']
        preTouchFramePoseStamped = self.makePoseStampedFromTouchFrame(preTouchFrame)
        preTouch_ik_response = self.robotService.runIK(preTouchFramePoseStamped, seedPose=above_table_pre_touch, nominalPose=above_table_pre_touch)

        if not preTouch_ik_response.success:
            rospy.loginfo("pre touch pose ik failed, returning")
            return False

        touchFramePoseStamped = self.makePoseStampedFromTouchFrame(touchFrame)
        preTouchPose = preTouch_ik_response.joint_state.position

        touch_ik_response = self.robotService.runIK(touchFramePoseStamped, seedPose=preTouchPose, nominalPose=preTouchPose)

        if not touch_ik_response.success:
            rospy.loginfo("touch pose not reachable, returning")
            return False

        touchPose = touch_ik_response.joint_state.position

        # store for future use
        self.preTouchFrame = preTouchFrame
        self.touchFrame = touchFrame
        self.robotService.moveToJointPosition(preTouchPose, maxJointDegreesPerSecond=params['speed']['pre_touch'])
        self.robotService.moveToJointPosition(touchPose, maxJointDegreesPerSecond=params['speed']['touch'])

        return True

    '''
    basic functions
    '''
    def moveHome(self):
        rospy.loginfo("moving home")
        homePose = self.touchParams['poses']['above_table_pre_touch']
        self.robotService.moveToJointPosition(homePose, maxJointDegreesPerSecond=self.touchParams['speed']['nominal'])

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
                                       'src', 'catkin_projects',
                                       'station_config', 'RLG_iiwa_1',
                                       'manipulation', 'params_touch.yaml')
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

    def testCollectSensorDataAndGenerateTouches(self):
        self.collectSensorData()
        self.requestTouch()
        # self.moveHome()
        result = self.waitForGenerateTouchesResult()
        self.processGenerateTouchesResult(result)

    def testPlanAndTouchObject(self):
        self.collectSensorData()
        self.requestTouch()
        self.moveHome()
        result = self.waitForGenerateTouchesResult()
        self.processGenerateTouchesResult(result)
        self.attemptTouch()
        self.moveHome()


def main():
    rospy.init_node('touch_supervisor_node')

    tfWrapper = TFWrapper()
    tfBuffer = tfWrapper.getBuffer()

    touchSupervisor = TouchSupervisor.makeDefault(tfBuffer=tfBuffer)
    touchSupervisor.testCollectSensorDataAndGenerateTouches()

    # rospy.spin()


if __name__ == "__main__":
    main()
