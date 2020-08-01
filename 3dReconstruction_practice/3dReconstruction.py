#!/usr/bin/env python
# -*- coding: utf-8 -

import rospy
from PIL import Image
import os 
from sensor_msgs.msg import PointCloud
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

def publishPointCloud(publisher, points):
	pc = PointCloud()
	pc.header.frame_id = "aruco_slam_world";
	for p in points:
		p32 = Point32()
		p32.x = p[0]
		p32.y = p[1]
		p32.z = p[2]
		pc.points.append(p32)
		
	publisher.publish(pc)

def main():
	rospy.init_node('3d_res', anonymous=False)
	orb = cv.ORB_create()
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
		kp = orb.detect(rgb_image, None)
		img = cv.drawKeypoints(rgb_image, kp, None, color=(0,255,0), flags=0)

		for k in kp:
			pt = k.pt
			s = getDepthOfCoordinate(depth_image, pt)
			h = convertToHomogeneous(pt)
			point3d = project2dto3d(h, iR, t, iM, s)
			points.append(point3d)

		publishPointCloud(publisher, points)
		cv.imshow("frame", img)
		cv.waitKey(10)

if __name__ == '__main__':
	main()
