import numpy as np
import cv2
import cv2.aruco as aruco
import math
"""
**************************************************************************
*                  E-Yantra Robotics Competition
*                  ================================
*  This software is intended to check version compatiability of open source software
*  Theme: Thirsty Crow
*  MODULE: Task1.1
*  Filename: detect.py
*  Version: 1.0.0  
*  Date: October 31, 2018
*  
*  Author: e-Yantra Project, Department of Computer Science
*  and Engineering, Indian Institute of Technology Bombay.
*  
*  Software released under Creative Commons CC BY-NC-SA
*
*  For legal information refer to:
*        http://creativecommons.org/licenses/by-nc-sa/4.0/legalcode 
*     
*
*  This software is made available on an “AS IS WHERE IS BASIS”. 
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*  
*  e-Yantra - An MHRD project under National Mission on Education using 
*  ICT(NMEICT)
*
**************************************************************************
"""

####################### Define Utility Functions Here ##########################
"""
Function Name : getCameraMatrix()
Input: None
Output: camera_matrix, dist_coeff
Purpose: Loads the camera calibration file provided and returns the camera and
		 distortion matrix saved in the calibration file.
"""


''' 
Used to detect the aruco markers and then overlay a cube, axis or a cylinder on the detected aruco. 
'''
def getCameraMatrix():
		with np.load('System.npz') as X:
				camera_matrix, dist_coeff, _, _ = [X[i] for i in ('mtx','dist','rvecs','tvecs')]
		return camera_matrix, dist_coeff

"""
Function Name : sin()
Input: angle (in degrees)
Output: value of sine of angle specified
Purpose: Returns the sine of angle specified in degrees
"""
def sin(angle):
		return math.sin(math.radians(angle))

"""
Function Name : cos()
Input: angle (in degrees)
Output: value of cosine of angle specified
Purpose: Returns the cosine of angle specified in degrees
"""
def cos(angle):
		return math.cos(math.radians(angle))



################################################################################


"""
Function Name : detect_markers()
Input: img (numpy array), camera_matrix, dist_coeff
Output: aruco list in the form [(aruco_id_1, centre_1, rvec_1, tvec_1),(aruco_id_2,
		centre_2, rvec_2, tvec_2), ()....]
Purpose: This function takes the image in form of a numpy array, camera_matrix and
		 distortion matrix as input and detects ArUco markers in the image. For each
		 ArUco marker detected in image, paramters such as ID, centre coord, rvec
		 and tvec are calculated and stored in a list in a prescribed format. The list
		 is returned as output for the function
"""
def detect_markers(img, camera_matrix, dist_coeff):
		markerLength = 100
		aruco_list = []
		aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
		parameters = aruco.DetectorParameters_create()

		# lists of ids and the corners beloning to each id
		corners, ids, rejectedImgPoints = aruco.detectMarkers(img, aruco_dict, parameters=parameters)
		# print(corners)
		cx = []
		cy = []
		for x in range(ids.size):
				rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners[x], markerLength, camera_matrix,dist_coeff)  # Estimate pose of each marker and return the values rvet and tvec---different from camera coefficients

				aruco.drawDetectedMarkers(img, corners)


				cx.append(int((corners[x][0][0][0] + corners[x][0][1][0] + corners[x][0][2][0] + corners[x][0][3][0]) / 4))
				cy.append(int((corners[x][0][0][1] + corners[x][0][1][1] + corners[x][0][2][1] + corners[x][0][3][1]) / 4))
				tup = (ids[x], (cx[x], cy[x]), rvec, tvec)  # Draw A square around the markers
				aruco_list.append(tup)
		#


		return aruco_list

"""
Function Name : drawAxis()
Input: img (numpy array), aruco_list, aruco_id, camera_matrix, dist_coeff
Output: img (numpy array)
Purpose: This function takes the above specified outputs and draws 3 mutually
		 perpendicular axes on the specified aruco marker in the image and
		 returns the modified image.
"""
def drawAxis(img, aruco_list, aruco_id, camera_matrix, dist_coeff):
		for x in aruco_list:
				if aruco_id == x[0]:
						rvec, tvec = x[2], x[3]
		markerLength = 100
		m = markerLength/2
		pts = np.float32([[-m,m,0],[m,m,0],[-m,-m,0],[-m,m,m]])
		# pts1 = np.float32([[m,-m,0],[-m,-m,0],[m,m,0],[m,-m,m]])

		pt_dict = {}
		imgpts, _ = cv2.projectPoints(pts, rvec, tvec, camera_matrix, dist_coeff)
		for i in range(len(pts)):
				 pt_dict[tuple(pts[i])] = tuple(imgpts[i].ravel())
		src = pt_dict[tuple(pts[0])];   dst1 = pt_dict[tuple(pts[1])];
		dst2 = pt_dict[tuple(pts[2])];  dst3 = pt_dict[tuple(pts[3])];
		img = cv2.line(img, src, dst1, (0,255,0), 4)
		img = cv2.line(img, src, dst2, (255,0,0), 4)
		img = cv2.line(img, src, dst3, (0,0,255), 4)
		# print(imgpts)

		return img

"""
Function Name : drawCube()
Input: img (numpy array), aruco_list, aruco_id, camera_matrix, dist_coeff
Output: img (numpy array)
Purpose: This function takes the above specified outputs and draws a cube
		 on the specified aruco marker in the image and returns the modified
		 image.
"""
def drawCube(img, ar_list, ar_id, camera_matrix, dist_coeff):
		for x in ar_list:
				if ar_id == x[0]:
						rvec, tvec = x[2], x[3]
		markerLength = 100
		m = markerLength/2
		######################## INSERT CODE HERE ########################


		##################################################################
		return img

"""
Function Name : drawCylinder()
Input: img (numpy array), aruco_list, aruco_id, camera_matrix, dist_coeff
Output: img (numpy array)
Purpose: This function takes the above specified outputs and draws a cylinder
		 on the specified aruco marker in the image and returns the modified
		 image.
"""
def drawCylinder(img, ar_list, ar_id, camera_matrix, dist_coeff):
		for x in ar_list:
				if ar_id == x[0]:
						rvec, tvec = x[2], x[3]
		markerLength = 100
		radius = markerLength/2; height = markerLength*1.5
		######################## INSERT CODE HERE ########################


		##################################################################
		return img

"""
MAIN CODE
This main code reads images from the test cases folder and converts them into
numpy array format using cv2.imread. Then it draws axis, cubes or cylinders on
the ArUco markers detected in the images.
"""


if __name__=="__main__":
		cam, dist = getCameraMatrix()
		img = cv2.imread("image_5.jpg")
		aruco_list = detect_markers(img, cam, dist)
		print(aruco_list)

		for i in aruco_list:
				img = drawAxis(img, aruco_list, i[0], cam, dist)
				##  img = drawCube(img, aruco_list, i[0], cam, dist)
				##  img = drawCylinder(img, aruco_list, i[0], cam, dist)
		cv2.imshow("img", img)
		cv2.waitKey(0)
		cv2.destroyAllWindows()

