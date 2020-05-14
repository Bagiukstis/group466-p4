#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Point
import numpy as np


rospy.init_node('talker')
pub = rospy.Publisher('chatter', Point, queue_size = 10)
rate = rospy.Rate(0.5)

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
	#print(A)	
	Final = np.round(Dotp, 3)
	#Final1 = np.array(Final, dtype=float)

	X_b = Final[0]
	Y_b = Final[1]
	Z_b = Final[2]
	return X_b, Y_b, Z_b
X_b, Y_b, Z_b = transformation(Xb, Yb, Zb)
X_c, Y_c, Z_c = transformation(Xc, Yc, Zc)

print('Bottle')
pb = Point()
pb.x = X_b
pb.y = Y_b
pb.z = Z_b
print(pb)

print('Cup')
pc = Point()
pc.x = X_c
pc.y = Y_c
pc.z = Z_c
print(pc)

while not rospy.is_shutdown():
	pub.publish(pb)
	pub.publish(pc)
	rate.sleep()
#print(X_b)
#print(Y_b)
#print(Z_b)