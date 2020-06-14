#! /usr/bin/env python
import rospy

from geometry_msgs.msg import PointStamped
import numpy as np

def transformation(x, y, z):
	A = np.array([[0.222, 0.335, -0.9162, 0.3607], [0.9750, -0.0760, 0.2088, -0.5391], [0, -0.9397, -0.3420, 0.3899], [0, 0, 0, 1]],dtype = np.float64)
	Pc = np.array([[x],[y],[z], [1]],dtype = np.float64)
	Dotp= A.dot(Pc)
	Final = np.round(Dotp, 3)

	X = Final[0]-0.35
	Y = Final[1]
	Z = Final[2]+0.25
	return X, Y, Z

def callback(message):
	X, Y, Z = transformation(message.point.x, message.point.y, message.point.z)
	rospy.loginfo(rospy.get_caller_id() + ": %s - x = %.3f, y = %.3f, z = %.3f", message.header.frame_id, X, Y, Z)
	pb = PointStamped()

	pb.header.frame_id = message.header.frame_id
	pb.point.x = X
	pb.point.y = Y
	pb.point.z = Z

	if not rospy.is_shutdown():
		pub.publish(pb)


if __name__ == '__main__':
	rospy.init_node('listenernpublisher')
	pub = rospy.Publisher('chatter', PointStamped, queue_size = 10)
	rospy.Subscriber("cv/object_coordinates", PointStamped, callback)
	rospy.spin()
