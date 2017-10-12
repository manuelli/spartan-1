#system
import numpy as np
import time
import os
import subprocess

#director
from director import lcmUtils
from director import lcmframe
from director import transformUtils
from director import robotsystem
from director import segmentation
from director import cameraview
from director import pydrakeik
from director import packagepath
from director import roboturdf
from director import robotlinkselector
from director import fieldcontainer
from director import framevisualization
from director import drcargs
from director import visualization as vis
from director import ikplanner
from director import robotposegui
from director.timercallback import TimerCallback


RobotPoseGUIWrapper = ikplanner.RobotPoseGUIWrapper
import bot_core as lcmbotcore
from director.ikparameters import IkParameters


try:
	from labelfusion.cameraposes import CameraPoses
except:
	print "Have you sourced setup_environment_spartan.sh in src/LabelFusion?"
	quit()

# spartan
import spartan.utils as spartanUtils
from spartan.taskrunner import TaskRunner

"""
To run this set useKukaRLGDev to True in iiwaManipApp.py. This loads a
HandEyeCalibration object into the director workspace, it is called cal.

To run calibration simply press F8 and enter cal.runCalibration(). This creates a new directory
in spartan/calibration_data which contains two files.

calibration.lcmlog: a log of the run
robot_data.yaml: relevant data for the "hand_link" poses during the run.

To finish the process go to the log folder mentioned above and run

run_elastic_fusion_on_log.py -l calibration.lcmlog
add_camera_poses_to_calibration_data.py

Then the file camera_poses_and_robot_data.yaml contains all the information
needed to run an AX=XB style hand-eye calibration.



"""

class RobotService(object):

	def __init__(self, robotSystem, removeFloatingBase=True):
		self.robotSystem = robotSystem
		self.manipPlanner = robotSystem.manipPlanner
		self.removeFloatingBase = True
		self.ikPlanner = robotSystem.ikPlanner

	"""
	Joint positions should be a dict of the form{'joint_name': joint_value}

	"""

	def movePose(self, joint_positions, maxDegreesPerSecond=30):
		
		assert isinstance(joint_positions, dict)

		self.manipPlanner.lastPlan = None
		startPose = self.robotSystem.robotStateJointController.q
		endPose = self.ikPlanner.mergePostures(startPose, joint_positions)
		ikParameters = IkParameters(maxDegreesPerSecond=maxDegreesPerSecond)
		plan = self.ikPlanner.computePostureGoal(startPose, endPose, ikParameters=ikParameters)
		self.manipPlanner.commitManipPlan(plan)

		# block until this plan is over
		planDuration = plan.plan[-1].utime/1e6
		print "running plan of duration %s seconds "  %(planDuration)
		time.sleep(1.1*planDuration)

	def getPose(self):
		q = self.robotSystem.robotStateJointController.q
		if self.removeFloatingBase:
			q = q[6:]

		return q
		

class HandEyeCalibration(object):

	def __init__(self, robotSystem, handFrame='palm'):
		self.robotSystem = robotSystem
		self.robotService = RobotService(robotSystem)
		self.handFrame = handFrame
		self.setup()
		self.calibrationData = None

		self.timer = TimerCallback(targetFps=1)
		self.timer.callback = self.callback
		self.task_runner = TaskRunner()
        # self.timer.callback = self.callback

	def setup(self):
		self.nominalPose = 'center'
		self.poseList = ['center', 'left', 'right', 'forwards', 'backwards']
		self.groupName = 'Calibration'
		poseNamesInCalibrationGroup = RobotPoseGUIWrapper.getPoseNamesInGroup(self.groupName)
		print "names in calibration group ", poseNamesInCalibrationGroup
		self.poseDict = dict()

		for poseName in self.poseList:
			d = dict()
			d['nominal'] = RobotPoseGUIWrapper.getPose(self.groupName, poseName)

			for extension in ['x', 'y', 'z']:
				poseNameExtended = poseName + "_" + extension
				if poseNameExtended in poseNamesInCalibrationGroup:
					d[extension] = RobotPoseGUIWrapper.getPose(self.groupName, poseNameExtended)

			self.poseDict[poseName] = d

		


	def captureDataAtPose(self, poseName=""):
		data = dict()
		data['pose_name'] = poseName
		data['pose'] = self.robotService.getPose().tolist()
		data['hand_frame_name'] = self.handFrame
		handTransform = self.robotSystem.robotStateModel.getLinkFrame(self.handFrame)
		data['hand_frame'] = spartanUtils.poseFromTransform(handTransform)
		data['utime'] = self.robotSystem.robotStateJointController.lastRobotStateMessage.utime
		return data

	def testCaptureData(self):
		data = self.captureDataAtPose()
		self.calibrationData.append(data)

	def moveHome(self):
		self.robotService.movePose(self.poseDict['center']['nominal'])

	def runThreaded(self):
		self.task_runner.call_on_thread(self.run)

	def run(self):
		self.calibrationData = []
		self.moveHome()

		for poseName in self.poseList:
			
			poses = self.poseDict[poseName]
			nominal = poses['nominal']
			self.robotService.movePose(nominal)
			data = self.captureDataAtPose('nominal')
			self.calibrationData.append(data)

			for subPoseName, pose in poses.iteritems():
				if subPoseName == 'nominal':
					continue;

				self.robotService.movePose(pose)
				data = self.captureDataAtPose(poseName + "_" + subPoseName)
				self.calibrationData.append(data)
				self.robotService.movePose(nominal)

			self.moveHome()
			

	def makePoseOrder(self):
		self.poseOrder = []
		for poseName in self.poseList:
			poses = self.poseDict[poseName]
			self.poseOrder.append(self.poseDict[poseName]['nominal'])
			for subPoseName, pose in poses.iteritems():
				if subPoseName == 'nominal':
					continue;
				self.poseOrder.append(self.poseDict[poseName][subPoseName])
			
			self.poseOrder.append(self.poseDict['center']['nominal'])

	def callback(self):
		self.testCaptureData()
		if len(self.poseOrder) == 0:
			print "finished data collection"
			self.timer.stop()
			self.loggerProcess.kill()
			self.saveCalibrationData(os.path.join(self.calibrationFolderName, 'robot_data.yaml'))
			return

		
		pose = self.poseOrder.pop(0)
		self.robotService.movePose(pose)


	def runCalibration(self):
		self.calibrationData = []
		unique_name = time.strftime("%Y%m%d-%H%M%S")
		self.calibrationFolderName = os.path.join(spartanUtils.getSpartanSourceDir(), 'calibration_data',unique_name)
		os.system("mkdir -p " + self.calibrationFolderName)
		os.chdir(self.calibrationFolderName)
		cmd = "lcm-logger calibration.lcmlog"
		self.loggerProcess = subprocess.Popen("exec " + cmd, shell = True)

		# waiting for logger to start
		print "waiting for logger to start . . . "
		time.sleep(2.0) # give the logger process time to start

		self.makePoseOrder()
		self.timer.start()

	def test(self):
		pose = self.poseDict['center']['nominal']
		self.robotService.movePose(pose)

	def saveCalibrationData(self, filename=None):
		if filename is None:
			filename = os.path.join(spartanUtils.getSpartanSourceDir(), 'sandbox', 'hand_eye_calibration_robot_data.yaml')


		spartanUtils.saveToYaml(self.calibrationData, filename)


"""
calibrationData is a list of dicts as above
cameraPoses is a CameraPoses object 
"""
def addCameraPosesToDataFile(calibrationData, cameraPoses):
	# augment data with matched camera poses
	# NOTE: camera poses are defined relative to first camera frame, hence the first one will look unintialized
	for index, value in enumerate(calibrationData):
		transform = cameraPoses.getCameraPoseAtUTime(value['utime'])
		(pos, quat) = transformUtils.poseFromTransform(transform)
		calibrationData[index]['camera_frame'] = dict()
		calibrationData[index]['camera_frame']['quaternion'] = dict()
		calibrationData[index]['camera_frame']['quaternion']['w'] = float(quat[0])
		calibrationData[index]['camera_frame']['quaternion']['x'] = float(quat[1])
		calibrationData[index]['camera_frame']['quaternion']['y'] = float(quat[2])
		calibrationData[index]['camera_frame']['quaternion']['z'] = float(quat[3])
		calibrationData[index]['camera_frame']['translation'] = dict()
		calibrationData[index]['camera_frame']['translation']['x'] = float(pos[0])
		calibrationData[index]['camera_frame']['translation']['y'] = float(pos[1])
		calibrationData[index]['camera_frame']['translation']['z'] = float(pos[2])

	return calibrationData

def processCalibrationData(posegraph_file, robot_data_filename, save_data_filename):
	cameraPoses = CameraPoses(posegraph_file)
	calibrationData = spartanUtils.getDictFromYamlFilename(robot_data_filename)
	calibrationData = addCameraPosesToDataFile(calibrationData, cameraPoses)
	spartanUtils.saveToYaml(calibrationData, save_data_filename)

