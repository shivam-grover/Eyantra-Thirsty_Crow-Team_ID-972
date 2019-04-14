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

import numpy as np
import cv2
import cv2.aruco as aruco
import math
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
from PIL import Image
import pygame
from objloader import *
import os, glob


texture_object = None
texture_background = None
camera_matrix = None
dist_coeff = None
cap = cv2.VideoCapture(0)
width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
INVERSE_MATRIX = np.array([[1.0, 1.0, 1.0, 1.0],
                           [-1.0, -1.0, -1.0, -1.0],
                           [-1.0, -1.0, -1.0, -1.0],
                           [1.0, 1.0, 1.0, 1.0]])
obj = None
crow = None
i = 1
################## Define Utility Functions Here #######################
"""
Function Name : getCameraMatrix()
Input: None
Output: camera_matrix, dist_coeff
Purpose: Loads the camera calibration file provided and returns the camera and
         distortion matrix saved in the calibration file.
"""


def getCameraMatrix():
    global camera_matrix, dist_coeff
    with np.load('Camera.npz') as X:
        camera_matrix, dist_coeff, _, _ = [X[i] for i in ('mtx', 'dist', 'rvecs', 'tvecs')]


########################################################################

############# Main Function and Initialisations ########################
"""
Function Name : main()
Input: None
Output: None
Purpose: Initialises OpenGL window and callback functions. Then starts the event
         processing loop.
"""


def main():

    glutInit()
    getCameraMatrix()
    glutInitWindowSize(640, 480)
    glutInitWindowPosition(625, 100)
    glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE)
    window_id = glutCreateWindow("OpenGL")
    init_gl()
    glutDisplayFunc(drawGLScene)
    glutIdleFunc(drawGLScene)
    glutReshapeFunc(resize)
    glutMainLoop()



"""
Function Name : init_gl()
Input: None
Output: None
Purpose: Initialises various parameters related to OpenGL scene.
"""


def init_gl():
    #load('aaaaaaaaaaaaaa')
    #load('cat')
    global crow
    global texture_object, texture_background
    glClearColor(0.0, 0.0, 0.0, 0.0)
    glClearDepth(1.0)
    glDepthFunc(GL_LESS)
    glEnable(GL_DEPTH_TEST)
    glShadeModel(GL_SMOOTH)
    glMatrixMode(GL_MODELVIEW)
    glEnable(GL_DEPTH_TEST)
    glEnable(GL_LIGHTING)
    glEnable(GL_LIGHT0)
    texture_background = glGenTextures(1)
    texture_object = glGenTextures(1)
    crow = OBJ('rocks.obj', swapyz=True)
    obj = OBJ('untitled.obj')
    print(obj)


"""
Function Name : resize()
Input: None
Output: None
Purpose: Initialises the projection matrix of OpenGL scene
"""


def resize(w, h):
    ratio = 1.0 * w / h
    glMatrixMode(GL_PROJECTION)
    glViewport(0, 0, w, h)
    gluPerspective(45, ratio, 0.1, 100.0)


"""
Function Name : drawGLScene()
Input: None
Output: None
Purpose: It is the main callback function which is called again and
         again by the event processing loop. In this loop, the webcam frame
         is received and set as background for OpenGL scene. ArUco marker is
         detected in the webcam frame and 3D model is overlayed on the marker
         by calling the overlay() function.
"""


def drawGLScene():
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    ar_list = []
    ret, frame = cap.read()
    if ret == True:
        draw_background(frame)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        ar_list = detect_markers(frame)
        if ar_list is not None:
            for i in ar_list:
                if i[0] == 0:
                    overlay(frame, ar_list, i[0], "texture_1.png")
                if i[0] == 2:
                    overlay(frame, ar_list, i[0], "texture_2.png")
                if i[0] == 1:
                    overlay(frame, ar_list, i[0], "texture_3.png")
                if i[0] == 6:
                    overlay(frame, ar_list, i[0], "texture_4.png")

        cv2.imshow('frame', frame)
        cv2.waitKey(1)
    glutSwapBuffers()


########################################################################

######################## Aruco Detection Function ######################
"""
Function Name : detect_markers()
Input: img (numpy array)
Output: aruco list in the form [(aruco_id_1, centre_1, rvec_1, tvec_1),(aruco_id_2,
        centre_2, rvec_2, tvec_2), ()....]
Purpose: This function takes the image in form of a numpy array, camera_matrix and
         distortion matrix as input and detects ArUco markers in the image. For each
         ArUco marker detected in image, paramters such as ID, centre coord, rvec
         and tvec are calculated and stored in a list in a prescribed format. The list
         is returned as output for the function
"""


def detect_markers(img):
    aruco_list = []
    ################################################################
    #################### Same code as Task 1.1 #####################
    ################################################################
    markerLength = 0.185
    aruco_list = []
    aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
    parameters = aruco.DetectorParameters_create()

    # lists of ids and the corners beloning to each id
    corners, ids, rejectedImgPoints = aruco.detectMarkers(img, aruco_dict, parameters=parameters)
    # print(corners)
    cx = []
    cy = []
    if ids is not None and corners is not None:
        for x in range(ids.size):
            rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners[x], markerLength, camera_matrix,
                                                            dist_coeff)  # Estimate pose of each marker and return the values rvet and tvec---different from camera coefficients
            ids = ids.astype('int64')
            aruco.drawDetectedMarkers(img, corners)

            cx.append(int(
                (corners[x][0][0][0] + corners[x][0][1][0] + corners[x][0][2][0] + corners[x][0][3][0]) / 4))
            cy.append(int(
                (corners[x][0][0][1] + corners[x][0][1][1] + corners[x][0][2][1] + corners[x][0][3][1]) / 4))
            tup = (ids[x, 0], (cx[x], cy[x]), rvec, tvec)  # Draw A square around the markers

            aruco_list.append(tup)
        #

        return aruco_list


########################################################################


################# This is where the magic happens !! ###################
############### Complete these functions as  directed ##################
"""
Function Name : draw_background()
Input: img (numpy array)
Output: None
Purpose: Takes image as input and converts it into an OpenGL texture. That
         OpenGL texture is then set as background of the OpenGL scene
"""

frames = []

def load(directory):
    os.chdir(directory)

    for file in glob.glob("*.obj"):
        frames.append(OBJ(file))

    os.chdir('..')
    frames_length = len(frames)



def next_frame():
    global i
    i += 1

    #if frame_index >= self.frames_length:
    if i >= len(frames):
        i = 0

    return frames[i].gl_list

def draw_background(img):
    glEnable(GL_TEXTURE_2D)

    bg_image = cv2.flip(img, 0)
    bg_image = Image.fromarray(bg_image)

    bg_image = bg_image.tobytes("raw", "RGB", 0, -1)

    # create background texture
    glBindTexture(GL_TEXTURE_2D, texture_background)
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST)
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_BGR, GL_UNSIGNED_BYTE, bg_image)
    glPushMatrix()
    i = 14.0
    # draw background
    glTranslatef(0.0, 0.0, -i)
    glBegin(GL_QUADS)
    glTexCoord2f(0.0, 1.0)
    glVertex3f(-(8*i/14), -(6*i/14), 0.0)
    glTexCoord2f(1.0, 1.0)
    glVertex3f((8*i/14), -(6*i/14), 0.0)
    glTexCoord2f(1.0, 0.0)
    glVertex3f((8*i/14), (6*i/14), 0.0)
    glTexCoord2f(0.0, 0.0)
    glVertex3f(-(8*i/14), (6*i/14), 0.0)
    glEnd()
    glPopMatrix()
    return None

"""
Function Name : init_object_texture()
Input: Image file path
Output: None
Purpose: Takes the filepath of a texture file as input and converts it into OpenGL
         texture. The texture is then applied to the next object rendered in the OpenGL
         scene.
"""


def init_object_texture(image_filepath):
    tex = cv2.imread(image_filepath)
    glEnable(GL_TEXTURE_2D)

    bg_image = cv2.flip(tex, 0)
    bg_image = Image.fromarray(bg_image)
    ix = bg_image.size[0]
    iy = bg_image.size[1]
    bg_image = bg_image.tobytes("raw", "BGRX", 0, -1)

    # create background texture

    glBindTexture(GL_TEXTURE_2D, texture_object)
    # glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    glPushMatrix()
    glTranslatef(0.0, 0.0, 10.0)
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)

    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST)

    # Draw textured Quads
    # glBegin(GL_QUADS)
    # glTexCoord2f(0.0, 0.0)
    # glVertex3f(0.0, 0.0, 0.0)
    # glTexCoord2f(1.0, 0.0)
    # glVertex3f(width, 0.0, 0.0)
    # glTexCoord2f(1.0, 1.0)
    # glVertex3f(width, height, 0.0)
    # glTexCoord2f(0.0, 1.0)
    # glVertex3f(0.0, height, 0.0)
    # glEnd()

    glTexImage2D(GL_TEXTURE_2D, 0, 3, ix, iy, 0, GL_RGBA, GL_UNSIGNED_BYTE, bg_image)
    glPopMatrix()

    return None


"""
Function Name : overlay()
Input: img (numpy array), aruco_list, aruco_id, texture_file (filepath of texture file)
Output: None
Purpose: Receives the ArUco information as input and overlays the 3D Model of a teapot
         on the ArUco marker. That ArUco information is used to
         calculate the rotation matrix and subsequently the view matrix. Then that view matrix
         is loaded as current matrix and the 3D model is rendered.

         Parts of this code are already completed, you just need to fill in the blanks. You may
         however add your own code in this function.
"""


def overlay(img, ar_list, ar_id, texture_file):
    for x in ar_list:
        if ar_id == x[0]:
            centre, rvec, tvecs = x[1], x[2], x[3]

    rmtx = cv2.Rodrigues(rvec)[0]
    offset = [[[0.185/4, -0.185/4, 0]]]
    tvecs = tvecs - offset
    font = cv2.FONT_HERSHEY_SIMPLEX #font for displaying text (below)

    cv2.putText(img, "Id: " + str(ar_id), centre, font, 1, (0,255,0),2,cv2.LINE_AA)

    # view_matrix = np.array([[rmtx[0][0], rmtx[0][1], rmtx[0][2], tvecs[0][0][0]],
    #                         [rmtx[1][0], rmtx[1][1], rmtx[1][2], tvecs[0][0][1]],
    #                         [rmtx[2][0], rmtx[2][1], rmtx[2][2], tvecs[0][0][2]],
    #                         [0.0, 0.0, 0.0, 1.0]])



    view_matrix = np.array([[rmtx[0][0], rmtx[0][1], rmtx[0][2], tvecs[0][0][0]*7.2],
                            [rmtx[1][0], rmtx[1][1], rmtx[1][2], tvecs[0][0][1]*7.2],
                            [rmtx[2][0], rmtx[2][1], rmtx[2][2], tvecs[0][0][2]*10],
                            [0.0, 0.0, 0.0, 1.0]])
    print(tvecs ,  texture_file)
    view_matrix = view_matrix * INVERSE_MATRIX
    view_matrix = np.transpose(view_matrix)
    init_object_texture(texture_file)

    glPushMatrix()
    glLoadMatrixd(view_matrix)
    #glutSolidTeapot(1.4)
    #glTranslate(0,0,1)
    glScale(0.2,0.2,0.2)

    glCallList(crow.gl_list)
    #glCallList(OBJ('pitcher3.obj').gl_list)
    #glCallList(next_frame())

    glPopMatrix()


########################################################################

if __name__ == "__main__":
    main()
