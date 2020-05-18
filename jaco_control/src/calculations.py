#! /usr/bin/env python
import rospy
#from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
import numpy as np

def callback(message):
	rospy.loginfo(rospy.get_caller_id() + "Header %s", message.header.frame_id)
	rospy.loginfo(rospy.get_caller_id() + "x = %.3f", message.point.x)
	rospy.loginfo(rospy.get_caller_id() + "y = %.3f", message.point.y)
	rospy.loginfo(rospy.get_caller_id() + "z = %.3f", message.point.z)
	if message.header.frame_id == "Bottle":
		Headb = message.header.frame_id
		Xb = message.point.x
		Yb = message.point.y
		Zb = message.point.z
	elif message.header.frame_id == "Mug":
		Headc= message.header.frame_id
		Xc = message.point.x
		Yc = message.point.y
		Zc = message.point.z
	def transformation(x, y, z):
		A = np.array([[0.222, 0.335, -0.9162, 0.3607], [0.9750, -0.0760, 0.2088, -0.5391], [0, -0.9397, -0.3420, 0.3899], [0, 0, 0, 1]],dtype = np.float64)
		Pc = np.array([[x],[y],[z], [1]],dtype = np.float64)
		Dotp= A.dot(Pc)
		Final = np.round(Dotp, 3)

		X = Final[0]
		Y = Final[1]
		Z = Final[2]
		return X, Y, Z
	X_b, Y_b, Z_b = transformation(Xb, Yb, Zb)
	X_c, Y_c, Z_c = transformation(Xc, Yc, Zc)
	pb = PointStamped()
	pb.header.frame_id = Headb
	pb.point.x = X_b
	pb.point.y = Y_b
	pb.point.z = Z_b
	
	pc = PointStamped()
	pc.header.frame_id = Headc
	pc.point.x = X_c
	pc.point.y = Y_c
	pc.point.z = Z_c
	while not rospy.is_shutdown():
		pub.publish(pb)
		pub.publish(pc)
		rate.sleep()


if __name__ == '__main__':
	rospy.init_node('listenernpublisher')
	rospy.Subscriber("cv/object_coordinates", PointStamped, callback)
	pub = rospy.Publisher('chatter', PointStamped, queue_size = 10)
	rospy.spin()

"""
rospy.init_node('talker')
pub = rospy.Publisher('chatter', PointStamped, queue_size = 10)
rate = rospy.Rate(0.5)




#message.header.frame_id, message.point.x, message.point.y, message.point.z = callback()

Xb = float(0*0.001)
Yb = float(0*0.001)
Zb = float(287.51 * 0.001)

Xc = float(0*0.001)
Yc = float(0*0.001)
Zc = float(222.51 * 0.001)

def transformation(x, y, z):
	A = np.array([[0.222, 0.335, -0.9162, 0.3607], [0.9750, -0.0760, 0.2088, -0.5391], [0, -0.9397, -0.3420, 0.3899], [0, 0, 0, 1]],dtype = np.float64)
	Pc = np.array([[x],[y],[z], [1]],dtype = np.float64)
	Dotp= A.dot(Pc)
	Final = np.round(Dotp, 3)

	X = Final[0]
	Y = Final[1]
	Z = Final[2]
	return X, Y, Z
X_b, Y_b, Z_b = transformation(Xb, Yb, Zb)
X_c, Y_c, Z_c = transformation(Xc, Yc, Zc)

pb = PointStamped()
pb.header.frame_id = "pipi"
pb.point.x = X_b
pb.point.y = Y_b
pb.point.z = Z_b
print(pb)

pc = PointStamped()
pc.point.x = X_c
pc.point.y = Y_c
pc.point.z = Z_c
print(pc)

while not rospy.is_shutdown():
	pub.publish(pb)
	#pub.publish(pc)
	rate.sleep()

"""