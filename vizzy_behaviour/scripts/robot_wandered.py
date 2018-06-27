#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
from time import sleep
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, PointStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import vizzy_msgs.msg
from random import randint
import woz_dialog_msgs.msg
from copy import deepcopy
import math
import yaml


class WayPoint:
    def __init__(self,**kargs):
    	self.gaze_frame = "l_camera_link"
    	self.goal = MoveBaseGoal()
        self.goal.target_pose.pose = Pose(Point(kargs['pose']['x_cord'], 
        									    kargs['pose']['y_cord'], 
        									    kargs['pose']['z_cord']), 
        								  Quaternion(kargs['pose']['quartinion_a'], 
        								  			 kargs['pose']['quartinion_b'], 
        								  			 kargs['pose']['quartinion_c'], 
        								  			 kargs['pose']['quartinion_d'])
        								  )

    	self.gaze = vizzy_msgs.msg.GazeGoal()
    	self.name = kargs['name']
    	self.speechString = kargs['speechString']
    	self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.header.stamp = rospy.Time.now()
    	self.gaze.type = vizzy_msgs.msg.GazeGoal.CARTESIAN
    	self.gaze.fixation_point_error_tolerance = 0.01
    	self.gaze.fixation_point.header.frame_id='l_camera_link'
        self.gaze.fixation_point.point.x = kargs['pose']['x_cord']
        self.gaze.fixation_point.point.y = -0.601618178873
        self.gaze.fixation_point.point.z = 0.224095496349
   

class RandomWalker():
	def callback(self, data):
		self.closestPerson = data.point

	def __init__(self):
		rospy.init_node('random_walker', anonymous=False)
		rospy.on_shutdown(self.shutdown)

		self.closestPerson = Point()
		self.closestPerson.x = 10.0
		self.closestPerson.y = 0.0
		self.closestPerson.z = 1.5
        
		with open("example.yaml", 'r') as stream:
			try:
			    points = yaml.load(stream)
			except yaml.YAMLError as exc:
			    print(exc)

		waypoints = [WayPoint(**point) for point in points]

		#Subscribe to closest points
		rospy.Subscriber("/closest_person", PointStamped, self.callback)

		self.gaze_active = rospy.get_param("~gaze_active", True)
		self.comlicenca_active = rospy.get_param("~comlicenca_active", True)
		self.beep_comlicenca_active = rospy.get_param("~beep_comlicenca_active", False)

		#Initialize
		self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist)
		self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
		self.move_base.wait_for_server(rospy.Duration(60))
		rospy.loginfo("Connected to move base server")

		self.gaze_client = actionlib.SimpleActionClient('gaze', vizzy_msgs.msg.GazeAction)
		self.gaze_client.wait_for_server()
		rospy.loginfo("Connected to gaze server")

		self.speech_client = actionlib.SimpleActionClient('nuance_speech_tts',woz_dialog_msgs.msg.SpeechAction)
		rospy.loginfo("Connected to speech server")
		rospy.loginfo("Starting to wander around")

		while not rospy.is_shutdown():
		    
			home_goal = vizzy_msgs.msg.GazeGoal()
			home_goal.type = vizzy_msgs.msg.GazeGoal.HOME
			self.gaze_client.send_goal(home_goal)

			i = randint(0, len(waypoints)-1)
			print('index: ' + str(i))
			way = waypoints[i]
			print('Moving to '+way.name)
			test = self.move(way)

			if not test:
			    continue

			print('gazing')
			self.gaze_client.send_goal(way.gaze)

			#self.gaze_client.wait_for_result()
			sleep(1) 
			speech_goal = woz_dialog_msgs.msg.SpeechGoal()
			speech_goal.voice = 'Joaquim'
			speech_goal.language = 'pt_PT'
			speech_goal.message ="silence5s"
			self.speech_client.send_goal(speech_goal)
			#self.speech_client.wait_for_result()
			sleep(5)
			speech_goal.message = way.speechString
			self.speech_client.send_goal(speech_goal)
			sleep(2)
			#self.speech_client.wait_for_result()


	def shutdown(self):
		rospy.loginfo("Stopping the robot...")
		self.move_base.cancel_goal()
		rospy.sleep(2)
		self.cmd_vel_pub.publish(Twist())
		rospy.sleep(1)


	def move(self, waypoint):

		self.move_base.send_goal(waypoint.goal)
		count_licenca = -1
		#While the robot does not get to the goal position randomly gaze at people
		while not self.move_base.get_state() == GoalStatus.SUCCEEDED and not self.move_base.get_state() == GoalStatus.LOST and not self.move_base.get_state() == GoalStatus.REJECTED and not self.move_base.get_state() == GoalStatus.ABORTED and not self.move_base.get_state() == GoalStatus.PREEMPTED and not rospy.is_shutdown():
			print('moving')
			sleep(0.3)
			#self.move_base.wait_for_result(rospy.Duration(1.0))
			print(math.sqrt(math.pow(self.closestPerson.x, 2)+math.pow(self.closestPerson.y, 2)))
		    #Gaze at people if gaze active
			if self.gaze_active and math.sqrt(math.pow(self.closestPerson.x, 2)+math.pow(self.closestPerson.y, 2)) < 5.0 and self.gaze_active and math.sqrt(math.pow(self.closestPerson.x, 2)+math.pow(self.closestPerson.y, 2)) > 1.5:
				print('gazing at person') 
				gaze_person = vizzy_msgs.msg.GazeGoal()
				gaze_person.type = vizzy_msgs.msg.GazeGoal.CARTESIAN
				gaze_person.fixation_point_error_tolerance = 0.01
				gaze_person.fixation_point.header.frame_id="base_footprint"
				gaze_person.fixation_point.point.x = self.closestPerson.x
				gaze_person.fixation_point.point.y = self.closestPerson.y
				gaze_person.fixation_point.point.z = self.closestPerson.z*1.03
				self.gaze_client.send_goal(gaze_person)
				#self.gaze_client.wait_for_result()
				sleep(1.7)
				gaze_person.type = vizzy_msgs.msg.GazeGoal.HOME
				self.gaze_client.send_goal(gaze_person)

				if self.move_base.get_state() == GoalStatus.ABORTED or self.move_base.get_state() == GoalStatus.PREEMPTED or self.move_base.get_state() == GoalStatus.REJECTED:
				    return False

			    
			if self.comlicenca_active and count_licenca < 0:
				if math.sqrt(math.pow(self.closestPerson.x, 2)+math.pow(self.closestPerson.y, 2)) < 2.2:
					speech_goal = woz_dialog_msgs.msg.SpeechGoal()
					self.closestPerson.x = 10.0
					self.closestPerson.y = 0.0
					self.closestPerson.z = 1.5
					speech_goal.voice = 'Joaquim'
					speech_goal.language = 'pt_PT'
					speech_goal.message ="silence1s"
					self.speech_client.send_goal(speech_goal)
					self.speech_client.wait_for_result()
				if not self.beep_comlicenca_active:
					speech_goal.message =	 "Com licença"
				else:
					speech_goal.message = "silence5s"
				self.speech_client.send_goal(speech_goal)
				self.speech_client.wait_for_result()
				sleep(1.0)
				count_licenca = 10

			count_licenca = count_licenca-1 
			self.closestPerson.x = 10.0
			self.closestPerson.y = 0.0
			self.closestPerson.z = 1.5 

		return True
		

if __name__ == '__main__':
    try:
        RandomWalker()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
