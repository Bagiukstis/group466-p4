import rospy
import cv2

import numpy as np

from sensor_msgs import point_cloud2 as pc2
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from geometry_msgs.msg import PointStamped
from image_geometry import PinholeCameraModel
from cv_bridge import CvBridge, CvBridgeError
from datetime import datetime


class ImageProcessor(object):
	def __init__(self):
		self.coordinate_publisher = rospy.Publisher('cv/object_coordinates', PointStamped)
		self._cam_info_sub = rospy.Subscriber('/pico_flexx/camera_info', CameraInfo, self.camera_info)
		self._camera_model = PinholeCameraModel()
		self._bridge = CvBridge()
		self.database_path = ["hu_mug.db", "hu_wine.db"]
		self.classifier = ImageClassifier(self.database_path)

		# Image containers for use outside of object
		self.ir_image_rect = None
		self.latest_point_cloud = None

	def camera_info(self, camera_info):
		rospy.loginfo(rospy.get_caller_id() + " :: PROJECTION MATRIX: %s", camera_info.P)
		self._camera_model.fromCameraInfo(camera_info)
		# Kills the subscriber, camera intrinsics are constant so 1 message is enough
		self._cam_info_sub.unregister()

	def undistort(self, src, dst):
		try:
			self._camera_model.rectifyImage(src, dst)
			return True

		except Exception:
			print(Exception)
			return False

	def ir_callback(self, image_frame):
		try:
			img = self._bridge.imgmsg_to_cv2(image_frame, desired_encoding='passthrough')
			self.ir_image_rect = np.zeros((img.shape[0], img.shape[1]), dtype='uint8')

			assert self.undistort(img, self.ir_image_rect), "Undistortion of IR img failed"
			result, segmented, objects, object_types = self.classifier.evaluate(self.ir_image_rect)
			# self.classifier.show(result, 500, 0, self.ir_image_rect)
			coordinates = self.get_position(objects)

			for i in range(0, len(coordinates)):
				message = PointStamped()
				message.point.x = coordinates[i][0]
				message.point.y = coordinates[i][1]
				message.point.z = coordinates[i][2]+0.04  # +0.04 to approximate the center of objects instead of the edge
				message.header.frame_id = object_types[i]
				self.coordinate_publisher.publish(message)

			# self.save_image(segmented, result, path='images/ir_images/')

		except CvBridgeError:
			print(CvBridgeError)

	def pcl_callback(self, point_cloud):
		self.latest_point_cloud = point_cloud
		return

	def save_image(self, img, img_rect, path='images/'):
		timestamp = str(datetime.now())
		try:
			ret1 = cv2.imwrite(path + 'raw/{}.jpg'.format(timestamp), img)
			ret2 = cv2.imwrite(path + 'rectified/{}-rect.jpg'.format(timestamp), img_rect)
			assert ret1 and ret2, "Failed saving images"
			rospy.loginfo(rospy.get_caller_id() + ": Images saved")

		except SystemError as e:
			print(e)

	def get_position(self, image_objects):
		try:
			object_coordinates = []
			for image_object in image_objects:
				m = cv2.moments(image_object)
				cx = int(m['m10'] / m['m00'])
				cy = int(m['m01'] / m['m00'])
				pcl = pc2.read_points(self.latest_point_cloud,
									  field_names=("x", "y", "z"),
									  skip_nans=True,
									  uvs=[[cx, cy]])
				for point in pcl:
					print(point[0], point[1], point[2])
					object_coordinates.append([point[0], point[1], point[2]])
			return object_coordinates
		except AssertionError as e:
			print(e)
			return []


class ImageClassifier():
	def __init__(self, database_path):
		self.training_data, self.category = self.load(database_path)
		self.knn = cv2.ml.KNearest_create()
		self.knn.train(np.float32(self.training_data), cv2.ml.ROW_SAMPLE, np.float32(self.category))

	def segment(self, img):
		k = 3
		ts_upper = 255
		ts_lower = 200
		gray_img = img  # cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
		ret, threshold = cv2.threshold(gray_img, ts_lower, ts_upper, cv2.THRESH_BINARY)
		kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
		close_img = cv2.morphologyEx(threshold, cv2.MORPH_CLOSE, kernel)
		open_img = cv2.morphologyEx(close_img, cv2.MORPH_OPEN, kernel)
		return open_img

	def representation(self, img):
		contours, hierarchy = cv2.findContours(img, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
		hu_moments = np.array([], np.float32)
		for contour in contours:
			hu_moments = np.append(hu_moments, np.array(cv2.HuMoments(cv2.moments(contour))[0:6], np.float32))
		hu_moments = hu_moments.reshape(int(hu_moments.shape[0] / 6), 6)
		return hu_moments, contours

	def load(self, path):
		samples = np.array([])
		group = np.array([])
		for file in range(len(path)):
			database = open(path[file], 'r')
			raw = database.read()
			database.close()
			items = raw.split("\n")
			# the list has to be count in reverse so that it doesnt run into a index error
			for item in range(len(items)):
				try:
					if "X" in items[item]:
						items.pop(item)
					else:
						entries = items[item].split(" ")
						entries.remove("")
						if entries:
							samples = np.append(samples, np.array(entries[0:6], np.float32))
							group = np.append(group, np.float32(file))
				except IndexError:
					pass
		return samples.reshape(int(samples.shape[0] / 6), 6), group

	def compare(self, img, items, contours):
		margin = 1
		min_area = 1000
		number_of_neighbours = 7
		recognized = []
		recognized_names = []

		ret, results, neighbours, dist = self.knn.findNearest(items, number_of_neighbours)
		for i in range(len(contours)):
			# boundingBox = cv2.minAreaRect(contours[contour])
			x, y, w, h = cv2.boundingRect(contours[i])
			box = cv2.boxPoints(((x + w / 2, y + h / 2), (w, h), 0.0))
			newbox = np.int0(box)
			horizontal = margin < x and ((x + w + margin) < img.shape[1])
			vertical = margin < y and (y + h + margin) < img.shape[0]
			area = min_area < h * w

			weird = 0
			name = ""
			color = (0, 0, 0)
			for n in range(len(neighbours[i])):
				if neighbours[i][n] == results[i]:
					weird += dist[i][n]
			if horizontal and vertical and area:
				if results[i] == 0:
					color = (255, 0, 0)
					name = 'Mug'
					recognized.append(contours[i])
					recognized_names.append(name)
				elif results[i] == 1:
					color = (255, 0, 255)
					name = 'Bottle'
					recognized.append(contours[i])
					recognized_names.append(name)
				elif results[i] == 2:
					color = (0, 0, 200)
				# if weird > .0008:
					# print('pretty weird ' + name + '!!!', weird, neighbours[i])
					# cv2.putText(img, 'XXXXXXX', (x, y), cv2.FONT_HERSHEY_PLAIN, 1, color, 1)
				cv2.drawContours(img, [newbox], 0, (70 * int(weird), 255, 70 * int(weird)), 1)
				cv2.putText(img, name, (x, y), cv2.FONT_HERSHEY_PLAIN, 1, color, 1)
				cv2.drawContours(img, [contours[i]], 0, color, 1)
			else:
				cv2.drawContours(img, [newbox], 0, (0, 0, 255), 1)
		return recognized, recognized_names

	def evaluate(self, img):
		seg = self.segment(img)
		items, contours = self.representation(seg)
		objects, object_types = self.compare(img, items, contours)
		return img, seg, objects, object_types

	# This function displays an image
	def show(self, img, width=500, time=0, img2=np.zeros((0, 0))):
		# The "width" is the value in pixels of the width a proportional resized version of the image to be displayed.
		# This is done so that the image can fit the screen, before it is displayed
		height = int(width * img.shape[0] / img.shape[1])
		resize = cv2.resize(img, (width, height))
		cv2.imshow('image 1', resize)
		if len(img2):
			height2 = int(width * img2.shape[0] / img2.shape[1])
			resize2 = cv2.resize(img2, (width, height2))
			cv2.imshow('image 2', resize2)
		# "time" is the waiting period, defined in milliseconds.
		# If time is set to 0 it will wait until a key is pressed,
		# before closing the window with the displayed image
		cv2.waitKey(time)


def main():
	rospy.init_node('cv', anonymous=True)
	image_processor = ImageProcessor()

	rospy.Subscriber('/pico_flexx/image_mono8', Image, image_processor.ir_callback)
	rospy.Subscriber("/pico_flexx/points", PointCloud2, image_processor.pcl_callback)
	# rospy.Subscriber('/pico_flexx/image_depth', Image, image_processor.depth_image_callback)

	try:
		rospy.spin()

	except KeyboardInterrupt:
		rospy.loginfo(rospy.get_caller_id() + ": Shutting down")

	cv2.destroyAllWindows()


def test_function():
	processor = ImageProcessor()
	classifier = ImageClassifier(["hu_mug.db", "hu_wine.db"])
	test_image = cv2.imread('/home/rns/Documents/Robotics/semester4/P4/p4-ws/images/ir_images/rectified/2020-04-29 16:02:20.852668-rect.jpg')
	img, segmented, blobs, blob_types = classifier.evaluate(test_image)
	processor.get_position(blobs)


if __name__ == '__main__':
	main()
	# test_function()
