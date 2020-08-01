#!/usr/bin/env python
# -*- coding: utf-8 -

import rospy
from PIL import Image
import os 
from sensor_msgs.msg import PointCloud, ChannelFloat32
from geometry_msgs.msg import Point32
import numpy as np
import cv2 as cv 

def getPathsFromFile(filename):
	file = open(filename, "r")
	paths = list()
	lines = file.readlines()
	lines = lines[3:]

	for line in lines:
		line = line[:-1]
		path = line.split(" ")[1]
		path = "/home/basestation/rover20_ws/src/rover_20/rover_20_image/scripts/rgbd_dataset_freiburg1_teddy/"+path
		paths.append(path)

	return paths

def getDepthOfCoordinate(depth_img, point, scale=5000):
	u = int(point[0])
	v = int(point[1])
	depth = depth_img.getpixel((u,v))
	depth = depth / float(scale)
	return depth

def convertToHomogeneous(cor):
	homogeneous = np.array([cor[0], cor[1], 1], dtype=np.float32)
	return homogeneous

def createIntrinsicParams():
	M = np.array([
		[517.3, 0, 318.6],
		[0, 516.5, 255.3],
		[0, 0, 1]
	], dtype=np.float32)
	return M

def createExtrinsicParams(raw=np.pi/2, pitch=0, yaw=0, x=0, y=0, z=0):
	rot_vec = np.array([raw,pitch,yaw], dtype=np.float32)
	R,_ = cv.Rodrigues(rot_vec)
	t_vec = np.array([x,y,z], dtype=np.float32)
	return R, t_vec

def project2dto3d(h, iR, t, iM, s):
	p_ = s * iM.dot(h)
	p_ = np.subtract(p_,t)
	point3d = iR.dot(p_)
	return point3d

def createPointCloudMsg(points, rgbs):
	pc_msg = PointCloud()
	pc_msg.header.frame_id = "aruco_slam_world";

	r_c = ChannelFloat32()
	g_c = ChannelFloat32()
	b_c = ChannelFloat32()

	r_c.name = "r"
	g_c.name = "g"
	b_c.name = "b"

	for i in range(len(points)):
		p = points[i]
		_p = Point32()
		_p.x = p[0]
		_p.y = p[1]
		_p.z = p[2]
		pc_msg.points.append(_p)

		rgb = rgbs[i]
		r = rgb[0]
		g = rgb[1]
		b = rgb[2]

		r_c.values.append(r)
		g_c.values.append(g)
		b_c.values.append(b)

	pc_msg.channels.append(r_c)
	pc_msg.channels.append(g_c)
	pc_msg.channels.append(b_c)
	return pc_msg

def main():
	rospy.init_node('3d_res_pixel', anonymous=False)
	depth_paths = getPathsFromFile("/home/basestation/rover20_ws/src/rover_20/rover_20_image/scripts/rgbd_dataset_freiburg1_teddy/depth.txt")
	rgb_paths = getPathsFromFile("/home/basestation/rover20_ws/src/rover_20/rover_20_image/scripts/rgbd_dataset_freiburg1_teddy/rgb.txt")
	publisher = rospy.Publisher("/pointCloud", PointCloud, queue_size=10)

	R,t = createExtrinsicParams()
	M = createIntrinsicParams()

	iM = np.linalg.inv(M)
	iR = np.linalg.inv(R)

	#changeable parameter
	max_frames = 500
	frames = min(max_frames, len(rgb_paths))

	for i in range(frames):
		depth_image = Image.open(depth_paths[i])
		rgb_image = cv.imread(rgb_paths[i])

		points = list()
		rgbs = list()

		print(rgb_image.shape)

		w = rgb_image.shape[1]
		h = rgb_image.shape[0]

		for row in range(h):
			for col in range(w):

				pt = (col, row)

				s = getDepthOfCoordinate(depth_image, pt)
				if s == 0:
					continue

				h = convertToHomogeneous(pt)
				point3d = project2dto3d(h, iR, t, iM, s)
				points.append(point3d)

				rgb = rgb_image[row, col, :]
				rgbs.append(tuple(rgb))

		msg = createPointCloudMsg(points, rgbs)
		publisher.publish(msg)

		cv.imshow("frame", rgb_image)
		cv.waitKey(10)

if __name__ == '__main__':
	main()
