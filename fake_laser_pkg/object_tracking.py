#!/usr/bin/env python
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
class Follower:
	x=200
	y=50
	h=20
	w=20
	def __init__(self):
		self.bridge = cv_bridge.CvBridge()
		#cv2.namedWindow("window", 1)
		self.image_sub = rospy.Subscriber('camera/rgb/image_raw',Image, self.image_callback)
		self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop',Twist, queue_size=1)
		self.twist = Twist()
	def image_callback(self, msg):
		image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
		#hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
		#lower_yellow = numpy.array([ 10, 10, 10])
		#upper_yellow = numpy.array([255, 255, 250])
		#mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
		h1, w1, d = image.shape
		#search_top = 3*h/4
		#search_bot = 3*h/4 + 20
		#mask[0:search_top, 0:w] = 0
		#mask[search_bot:h, 0:w] = 0
		#M = cv2.moments(mask)
		#if M['m00'] > 0:
		#cx = int(M['m10']/M['m00'])
		#cy = int(M['m01']/M['m00'])
		if(self.x<500):
			self.x+=10
		else:
			self.x=200
		if(self.y<500):
			self.y+=10
		else:
			self.y=50
		if(self.h<50):
			self.h+=1
		else:
			self.h=20
		if(self.w<50):
			self.w+=1
		else:
			self.w=20
		cx=w1/2
		#cx=self.x+self.w/2
		#cy=self.y+self.h/2
		cy=h1/2
		cv2.circle(image, (cx, cy), self.w/2, (0,0,255), -1)
		err = cx - w1/2
		self.twist.linear.x = 0.2
		self.twist.angular.z = -float(err) / 100
		self.cmd_vel_pub.publish(self.twist)
		cv2.imshow("window", image)
		cv2.waitKey(3)
rospy.init_node('follower')
follower = Follower()
rospy.spin()
