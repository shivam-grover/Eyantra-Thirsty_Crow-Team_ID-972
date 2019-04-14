

import numpy as np
import cv2
import cv2.aruco as aruco
import math
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
from PIL import Image
import time
from objloader import *
import serial
import threading
import os, glob


texture_object = None
texture_background = None
camera_matrix = None
dist_coeff = None

cap = cv2.VideoCapture(1)
width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))      #to get the width of the frame
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))    #to get the height of the frame
INVERSE_MATRIX = np.array([[1.0, 1.0, 1.0, 1.0],
                           [-1.0, -1.0, -1.0, -1.0],
                           [-1.0, -1.0, -1.0, -1.0],
                           [1.0, 1.0, 1.0, 1.0]])

##Dictionaries to define the arena
adjV = {}                  #A dictionary to store the vertices and their relation(edges) with other vertices
                           #for each vertex as the key, the value(list) are the adjacent vertices to the key
for i in range(1, 55):
    adjV[i] = []


cell = {}                  #A dictionary to store the cell Numbers and their relation to the vertices adjacent
                           #to it and axes
for i in range(1, 20):
    cell[i] = []


edge_Axis = {}              #A dictionary to store the edges and the axes they're parallel to
for i in range(55):
    edge_Axis[i] = []

START = 0
Robot_start = ""
#OBJ FILES#
crow = None
pebble = None               #for the initial pebble stack
pebble_dim = None           #for the final pebble stack

waterE = None           #for the initial water pitcher
waterM1 = None           #for the middle water pitcher 1
waterM2 = None           #for the middle water pitcher 2

waterF = None           #for the filled water pitcher
global flag


#Triggers for animation
flag = 0                #to trigger animation for the water pitcher and crow
flaga = 0               #to trigger animation for Pebblle 1
flagb = 0               #to trigger animation for Pebblle 2
flagc = 0               #to trigger animation for Pebblle 3
bend = 0                #to trigger animation for the bending of the crow after the pitcher is full to signify drinking
CrowPick = 0            #to trigger animation for the crow picking up the pebble in it's beak
CrowBeakClose = 1       #to trigger animation for the crow dropping the pebble
i = 1
frames = []             #to store the obj files for crow animation
frames_bend = []             #to store the obj files for crow animation of the crow bending its neck

frames_crowrock = []    #to store the obj files for crow animation with pebble

################DICTIONARY INDEX NUMBERS FOR EACH PEBBLE AND WATER PITCHER#########
P1=0
P2=0
P3=0
W=0


##Aruco IDs for pebbles
PebbleAR1 = P1    #Initially following the order of pebbles they were given in the dictionary. This will be changed later
                #in the function background() if the path of traversal would be shorter for a different order of pickup
                #We will use these variables for projecting and animating the right model on the right aruco object.
PebbleAR2 = P2
PebbleAR3 = P3
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
Function Name : foreground()
Input: None
Output: None
Purpose: Initialises OpenGL window and callback functions. Then starts the event
         processing loop.
"""

def foreground():
    glutInit()
    global flag
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
Function Name : main()
Input: None
Output: None
Purpose: assigns threads to foreground and background and starts them. We used threading coz of the multiple infinite loops(one in the openGL
				part and one for recieving trigger flags from the robot after pickups and drops)
"""

def main():
    a = threading.Thread(name='foreground', target=foreground)				#the openGL part
    b = threading.Thread(name='background', target=background)				#the path planning part and communication with the bot

    b.start()
    a.start()

#################################################################################################3

"""
Function Name : add_edge()
Input: dict (dictionary for adjacent vertices), V1, V2(int)
Output: None
Purpose: make Vertex V1 and V2 adjacent
"""

def add_edge(dict, V1, V2):
    dict[V1].append(V2)
    dict[V2].append(V1)
    #print("edge between: ", V1, V2)

"""
Function Name : add_edge_to_cell()
Input: dict (dictionary for relation between cells and vertices), node11, node12, node21, node22, node31, node32(int)
Output: None
Purpose: assigns vertices to the cell. the first two vertices lie on axis 1-1, next two on axis 2-2, and the later two on axis 3-3
"""

def add_edge_to_cell(dict, cell, node11, node12, node21, node22, node31, node32):
    dict[cell].append(node11)
    dict[cell].append(node12)
    dict[cell].append(node21)
    dict[cell].append(node22)
    dict[cell].append(node31)
    dict[cell].append(node32)

"""
Function Name : cellToNode()
Input: dict (dictionary for relation between cells and vertices), cellNo, axis(int)
Output: None
Purpose: Gives two possible destination vertices Based on the cellNo and axis
"""

def cellToNode(dict, cellNo, axis):
    if (axis == 1):
        N1 = dict[cellNo][0]
        N2 = dict[cellNo][1]
    elif(axis==2):
        N1 = dict[cellNo][2]
        N2 = dict[cellNo][3]
    elif(axis==3):
        N1 = dict[cellNo][4]
        N2 = dict[cellNo][5]

    return N1,N2

"""
Function Name : edgeAxis()
Input: dictE (dictionary for relation between edges(two adjacent vertices) and the axis they are parallel to), V1, V2, axis(int)
Output: None
Purpose: Assigns axis to a pair of adjacent vertices
"""

def edgeAxis(dictE_A, V1, V2, axis):
    dictE_A[V1][V2] = axis
    # print("axis between:", V1, V2, axis)
    dictE_A[V2][V1] = axis

"""
Function Name : printShortestDistance()
Input: adjV (dictionary for adjacent vertices), s(source vertex), dest(destination vertex)
Output: A list 'path' which contains the vertices that need to be traversed to go from
        s to dest
Purpose: Uses a modified version of BFS algorithm (and stores the predecessor as well which is used to give the path)
        and returns and prints the shortest path as a list
"""

def printShortestDistance(adjV, s, dest):

    pred = []
    dist = []
    path = []
    if(s == dest):
        path.append(s)
    else:
        pred = BFS(adjV,s,dest)
        crawl = dest
        path.append(crawl)
        while (pred[crawl] != -1):
            path.append(pred[crawl])
            crawl = pred[crawl]
    # print(pred)



    path.reverse()
    print("path is: ", path)
    return path

"""
Function Name : BFS()
Input: adjV (dictionary for adjacent vertices), src(source vertex), dest(destination vertex)
Output: A list 'pred' which contains the value of the predecessor vertex for the index as the current vertex
Purpose: A modified version of BFS algorithm (and stores the predecessor as well which is used to give the path)
        to find the shortest path and return a list pred which contains the value of the predecessor vertex for the index as the current vertex
"""

def BFS(adjV, src, dest):
    qu = []
    visited = [False]*55
    dist = [sys.maxsize]*55
    pred = [-1]*55
    # for i in range(1,55):
    #     dist[i] = sys.maxsize
    #     pred[i] = -1


    visited[src] = True                 # setting source as visited
    dist[src] = 0                       #distance of source from source is 0
    qu.append(src)                      #appending source to the queue

    while(qu):
        u = qu.pop(0)                   #storing the value before popping
        for i in range(len(adjV[u])):
            if(visited[adjV[u][i]]==False):     #only processing if unvisited
                visited[adjV[u][i]]=True        #set as visited
                dist[adjV[u][i]] = dist[u]+1    #set distance one greater than the pred
                pred[adjV[u][i]] = u            #set predecessor
                qu.append(adjV[u][i])           #append current node in queue

                if(adjV[u][i]==dest):
                    return pred

    return False

"""
Function Name : pathToAxis()
Input: path (a list containing vertices that need to be travelled), edge_Axis(a list which converts a pair of adjacent vertices, basically an edge, to the axis it is parallel)
Output: pathinAxis(A list which has the path to be taversed represented in axis form)
Purpose: converts a path list into an axis list. Since each axis is at 30 degrees angle in a hexagon, we can use axis to find whether to turn left or right at nodes
"""

def pathToAxis(path,edge_Axis):
    pathinAxis = []
    if(len(path)-1>0):
        for i in range(len(path)-1):
            # print(len(path), path[i], path[i+1])
            pathinAxis.append(edge_Axis[path[i]][path[i+1]])
    else:
        pathinAxis.append(2)
    return pathinAxis



"""
Function Name : axisToIns()
Input: axisIns (a list containing path in terms of axis from the start node to the pebble), 
        axisPathWaterpi ((a list containing path in terms of axis from the last traversed pebble to the water pitcher) 
        axisP (int, the axis at which the current pebble to be traversed to is aligned)
        axisWater(int, the axis at which the water pitcher is aligned)
        startAxis (int, the axis of the bot right before the function called)
        final(int, to check if this is the final pebble pickup/drop, 0 for no and 1 for yes)
        first(int, to check if this is the first pebble pickup, 0 for no and 1 for yes)
Output: ins (A list which has the final instructions to be sent to the robot for the given pebble)
Purpose: converts an axis list to instructions consisting of characters, each denoting a certain function:
         'r' : turn right by 30 degrees
         'l' : turn left by 30 degrees
         's' : stop and pick pebble
         'd' : stop and drop pebble
         'a' : turn 180 degrees
         'q' : turn right by 60 degrees to align the axis of the robot and the AR object
         'w' : turn left by 60 degrees to align the axis of the robot and the AR object
         'f' : stop and drop the pebble and ring the buzzer and exit
         
"""


def axisToIns(axisIns,axisPathWaterpi, startAxis,axisP,axisWater,final,first):
    ins = []        #ARRAY TO STORE INSTRUCTIONS

    #First turn for the pebble

    if(startAxis==2 and axisIns[0]== 3):
        ins.append('r')
    elif(startAxis==2 and axisIns[0]==1):
        ins.append('l')
    elif (startAxis == 1 and axisIns[0] == 3):
        ins.append('l')
    elif (startAxis == 3 and axisIns[0] == 1):
        ins.append('r')
    elif (startAxis == 3 and axisIns[0] == 2):
        ins.append('l')
    elif (startAxis == 1 and axisIns[0] == 2):
        ins.append('r')
    elif (startAxis == axisIns[0]):
        if(first==1):
            ins.append('s')
        else:
            ins.append('a')


######   PATH FOR FIRST PEBBLE    #######
    if (len(axisIns)-1>0):                                      #since the first turn is already decided, if the path
                                                                #has only one turn before the destination is reached
                                                                #we don't go through this for loop
        for i in range(1,len(axisIns)):
            if(axisIns[i-1]==1 and axisIns[i]==3):
                ins.append('l')
            elif (axisIns[i-1] == 3 and axisIns[i] == 1):
                ins.append('r')
            elif(axisIns[i-1]==2 and axisIns[i]==3):
                ins.append('r')
            elif (axisIns[i-1] == 3 and axisIns[i] == 2):
                ins.append('l')
            elif(axisIns[i-1]==1 and axisIns[i]==2):
                ins.append('r')
            elif (axisIns[i-1] == 2 and axisIns[i] == 1):
                ins.append('l')
            elif(axisIns[i-1]==axisIns[i]):
                ins.append('b')

            ######   ALIGNMENT WITH THE AXES    #######

            if(i == len(axisIns)-1):
                if(axisIns[i]==axisP):
                    ins.append('s')
                elif(axisIns[i]==2 and axisP==1):
                    ins.append('q')                 #right by 60 degrees
                    ins.append('s')
                    axisIns.append(1)
                elif(axisIns[i]==1 and axisP == 2):
                    ins.append('w')                 #left by 60 degrees
                    ins.append('s')
                    axisIns.append(2)
                elif(axisIns[i]==3 and axisP==2):
                    ins.append('q')                 #left by 60 degrees
                    ins.append('s')
                    axisIns.append(2)
                elif (axisIns[i] == 2 and axisP == 3):
                    ins.append('w')
                    ins.append('s')
                    axisIns.append(3)
                elif (axisIns[i] == 1 and axisP == 3):
                    ins.append('q')
                    ins.append('s')
                    axisIns.append(3)
                elif (axisIns[i] == 3and axisP == 1):
                    ins.append('w')
                    ins.append('s')
                    axisIns.append(1)
    else:
        if first == 0:
            i =0
            if(axisIns[i]==axisP):
                ins.append('s')
            elif(axisIns[i]==2 and axisP==1):
                ins.append('q')                 #right by 60 degrees
                ins.append('s')
                axisIns.append(1)
            elif(axisIns[i]==1 and axisP == 2):
                ins.append('w')                 #left by 60 degrees
                ins.append('s')
                axisIns.append(2)
            elif(axisIns[i]==3 and axisP==2):
                ins.append('q')                 #left by 60 degrees
                ins.append('s')
                axisIns.append(2)
            elif (axisIns[i] == 2 and axisP == 3):
                ins.append('w')
                ins.append('s')
                axisIns.append(3)
            elif (axisIns[i] == 1 and axisP == 3):
                ins.append('q')
                ins.append('s')
                axisIns.append(3)
            elif (axisIns[i] == 3and axisP == 1):
                ins.append('w')
                ins.append('s')
                axisIns.append(1)

    ######   FIRST TURN FOR WATER PITCHER    #######

    if(axisIns[len(axisIns)-1]==axisPathWaterpi[0]):
        ins.append('a')
    elif(axisIns[len(axisIns)-1]==1 and axisPathWaterpi[0]==2):
        ins.append('r')
    elif(axisIns[len(axisIns)-1]==2 and axisPathWaterpi[0]==1):
        ins.append('l')
    elif(axisIns[len(axisIns)-1]==1 and axisPathWaterpi[0]==3):
        ins.append('l')
    elif(axisIns[len(axisIns)-1]==3 and axisPathWaterpi[0]==1):
        ins.append('r')
    elif(axisIns[len(axisIns)-1]==2 and axisPathWaterpi[0]==3):
        ins.append('r')
    elif(axisIns[len(axisIns)-1]==3 and axisPathWaterpi[0]==2):
        ins.append('l')

    ######   PATH FOR WATER PITCHER    #######

    if((len(axisPathWaterpi)-1)>0):                             #since the first turn is already decided, if the path
                                                                #has only one turn before the destination is reached
                                                                #we don't go through this for loop
        for i in range(1,len(axisPathWaterpi)):

            if(axisPathWaterpi[i-1]==1 and axisPathWaterpi[i]==3):
                ins.append('l')
            elif (axisPathWaterpi[i-1] == 3 and axisPathWaterpi[i] == 1):
                ins.append('r')
            elif(axisPathWaterpi[i-1]==2 and axisPathWaterpi[i]==3):
                ins.append('r')
            elif (axisPathWaterpi[i-1] == 3 and axisPathWaterpi[i] == 2):
                ins.append('l')
            elif(axisPathWaterpi[i-1]==1 and axisPathWaterpi[i]==2):
                ins.append('r')
            elif (axisPathWaterpi[i-1] == 2 and axisPathWaterpi[i] == 1):
                ins.append('l')
            elif(axisPathWaterpi[i-1]==axisPathWaterpi[i]):
                ins.append('b')

            ######   ALIGNMENT OF AXES    #######

            if(i == len(axisPathWaterpi)-1):


                if(final == 0):                                                  #checking if final pebble
                    if(axisPathWaterpi[i]==axisWater):
                        ins.append('d')
                    elif (axisPathWaterpi[i] == 2 and axisWater == 1):
                        ins.append('q')  # left by 60 degrees
                        ins.append('d')
                    elif (axisPathWaterpi[i] == 1 and axisWater == 2):
                        ins.append('w')  # left by 60 degrees
                        # ins.append('q')  # right by 60 degrees
                        ins.append('d')
                    elif (axisPathWaterpi[i] == 3 and axisWater == 2):
                        ins.append('q')  # right by 60 degrees
                        ins.append('d')
                    elif (axisPathWaterpi[i] == 2 and axisWater == 3):
                        ins.append('w')  # left by 60 degrees
                        ins.append('d')
                    elif (axisPathWaterpi[i] == 1 and axisWater == 3):
                        ins.append('q')
                        ins.append('d')
                    elif (axisPathWaterpi[i] == 3 and axisWater == 1):
                        ins.append('w')
                        ins.append('d')
                else:
                    if (axisPathWaterpi[i] == axisWater):
                        ins.append('f')
                    elif (axisPathWaterpi[i] == 2 and axisWater == 1):
                        ins.append('q')  # left by 60 degrees
                        ins.append('f')
                    elif (axisPathWaterpi[i] == 1 and axisWater == 2):
                        ins.append('w')  # left by 60 degrees
                        # ins.append('q')  # right by 60 degrees
                        ins.append('f')
                    elif (axisPathWaterpi[i] == 3 and axisWater == 2):
                        ins.append('q')  # right by 60 degrees
                        ins.append('f')
                    elif (axisPathWaterpi[i] == 2 and axisWater == 3):
                        ins.append('w')  # left by 60 degrees
                        ins.append('f')
                    elif (axisPathWaterpi[i] == 1 and axisWater == 3):
                        ins.append('q')
                        ins.append('f')
                    elif (axisPathWaterpi[i] == 3 and axisWater == 1):
                        ins.append('w')
                        ins.append('f')
    else:
        i= 0
        if (final == 0):                                                #checking if final pebble
            if (axisPathWaterpi[i] == axisWater):
                ins.append('d')
            elif (axisPathWaterpi[i] == 2 and axisWater == 1):
                ins.append('q')  # left by 60 degrees
                ins.append('d')
            elif (axisPathWaterpi[i] == 1 and axisWater == 2):
                ins.append('w')  # left by 60 degrees
                # ins.append('q')  # right by 60 degrees
                ins.append('d')
            elif (axisPathWaterpi[i] == 3 and axisWater == 2):
                ins.append('q')  # right by 60 degrees
                ins.append('d')
            elif (axisPathWaterpi[i] == 2 and axisWater == 3):
                ins.append('w')  # left by 60 degrees
                ins.append('d')
            elif (axisPathWaterpi[i] == 1 and axisWater == 3):
                ins.append('q')
                ins.append('d')
            elif (axisPathWaterpi[i] == 3 and axisWater == 1):
                ins.append('w')
                ins.append('d')
        else:
            if (axisPathWaterpi[i] == axisWater):
                ins.append('f')
            elif (axisPathWaterpi[i] == 2 and axisWater == 1):
                ins.append('q')  # left by 60 degrees
                ins.append('f')
            elif (axisPathWaterpi[i] == 1 and axisWater == 2):
                ins.append('w')  # left by 60 degrees
                # ins.append('q')  # right by 60 degrees
                ins.append('f')
            elif (axisPathWaterpi[i] == 3 and axisWater == 2):
                ins.append('q')  # right by 60 degrees
                ins.append('f')
            elif (axisPathWaterpi[i] == 2 and axisWater == 3):
                ins.append('w')  # left by 60 degrees
                ins.append('f')
            elif (axisPathWaterpi[i] == 1 and axisWater == 3):
                ins.append('q')
                ins.append('f')
            elif (axisPathWaterpi[i] == 3 and axisWater == 1):
                ins.append('w')
                ins.append('f')

    print(ins)
    return ins

#################################################################################################3


"""
Function Name : calculatePath()
Input: arena_config (a dictionary contaning  the position, Aruco ID and axes of each Aruco object
       W1 (int, the cell number of the water pitcher)
       P1 (int, the cell number of the first pebble)
       W1 (int, the cell number of the second pebble)
       W1 (int, the cell number of the third pebble)    #P1,P2,P3 are not same as the P1, P2, P3 in the next function 

Output: Path (a list containing instructions(turn left, right etc) for the current order and configuration of objects) 
Purpose: Finds the final path for the bot in the following order:
        1. Finds the path between Robot_start to P1 and then from P1 to W1
        2. Finds the path between W1 to P2 and then from P2 to W1
        3. Finds the path between W1 to P3 and then from P3 to W1
        
        Then it adds the above paths and returns it
"""

def calculatePath(arena_config, W1, P1, P2, P3, Robot_start):
    global PebbleAR1,PebbleAR2,PebbleAR3
    if(arena_config[P1][2]=="1-1"):
        axis = 2
    elif (arena_config[P1][2] == "2-2"):
        axis = 3
    if (arena_config[P1][2] == "3-3"):
        axis = 1

    if (arena_config[W1][2] == "1-1"):
        axisWater = 2
    elif (arena_config[W1][2] == "2-2"):
        axisWater = 3
    if (arena_config[W1][2] == "3-3"):
        axisWater = 1

    if (arena_config[P2][2] == "1-1"):
        axis_Pebble2 = 2
    elif (arena_config[P2][2] == "2-2"):
        axis_Pebble2 = 3
    if (arena_config[P2][2] == "3-3"):
        axis_Pebble2 = 1

    if (arena_config[P3][2] == "1-1"):
        axis_Pebble3 = 2
    elif (arena_config[P3][2] == "2-2"):
        axis_Pebble3 = 3
    if (arena_config[P3][2] == "3-3"):
        axis_Pebble3 = 1

    cellNo = arena_config[P1][1]
    cellNo_pebble2 = arena_config[P2][1]
    cellNo_pebble3 = arena_config[P3][1]

    water = arena_config[W1][1]


    if(Robot_start == "START - 1"):
        source = 1
    elif (Robot_start == "START - 2"):
        source = 16


    PN1, PN2 = cellToNode(cell, cellNo, axis)                  #getting the two possible destination nodes for pebble 1
    P2N1,P2N2 = cellToNode(cell, cellNo_pebble2, axis_Pebble2) #getting the two possible destination nodes for pebble 2
    P3N1,P3N2 = cellToNode(cell, cellNo_pebble3, axis_Pebble3) #getting the two possible destination nodes for pebble 3
    WN1, WN2 = cellToNode(cell, water, axisWater)       #getting the two possible destination nodes for water pitcher
    pathN1 = printShortestDistance(adjV, source, PN1)   #path for pebble 1 at Node 1
    pathN2 = printShortestDistance(adjV, source, PN2)   #path for pebble 1 at Node 2

    if (len(pathN1) < len(pathN2)):
        print("We choose path N1")                      #deciding which of the two nodes for pebble is closer
        choice = pathN1
    elif (len(pathN2) < len(pathN1)):
        print("We choose path N2")
        choice = pathN2
    else:
        print("Both distances are same")
        choice = pathN1

    pathW1 = printShortestDistance(adjV, choice[len(choice) - 1], WN1)  #path for Water Pitcher from Pebble 1
    pathW2 = printShortestDistance(adjV, choice[len(choice) - 1], WN2)

    if (len(pathW1) < len(pathW2)):
        print("We choose path W1")                      #deciding which of the two nodes for water pitcher is closer
        choiceW = pathW1
    elif (len(pathW2) < len(pathW1)):
        print("We choose path W2")
        choiceW = pathW2
    else:
        print("Both distances are same")
        choiceW = pathW1

    print("DistanceP:", len(choiceW)+len(choice))

    pathP21 = printShortestDistance(adjV, choiceW[len(choiceW) - 1], P2N1)          #WATER PICTHER TO PEBBLE!
    pathP22 = printShortestDistance(adjV, choiceW[len(choiceW) - 1], P2N2)

    if (len(pathP21) < len(pathP22)):
        print("We choose path P21")                      #deciding which of the two nodes for water pitcher is closer
        choiceP2 = pathP21
    elif (len(pathP22) < len(pathP21)):
        print("We choose path P22")
        choiceP2 = pathP22
    else:
        print("Both distances are same")
        choiceP2 = pathP21



    pathW21 = printShortestDistance(adjV, choiceP2[len(choiceP2) - 1], WN1)     #path for Water Pitcher from Pebble 2
    pathW22 = printShortestDistance(adjV, choiceP2[len(choiceP2) - 1], WN2)

    if (len(pathW21) < len(pathW22)):
        print("We choose path W1")  # deciding which of the two nodes for water pitcher is closer
        choiceW2 = pathW21
    elif (len(pathW22) < len(pathW21)):
        print("We choose path W2")
        choiceW2 = pathW22
    else:
        print("Both distances are same")
        choiceW2 = pathW21


    pathP31 = printShortestDistance(adjV, choiceW2[len(choiceW2) - 1], P3N1)
    pathP32 = printShortestDistance(adjV, choiceW2[len(choiceW2) - 1], P3N2)

    if (len(pathP31) < len(pathP32)):
        print("We choose path P31")  # deciding which of the two nodes for water pitcher is closer
        choiceP3 = pathP31
    elif (len(pathP32) < len(pathP31)):
        print("We choose path P32")
        choiceP3 = pathP32
    else:
        print("Both distances are same")
        choiceP3 = pathP31

    pathW31 = printShortestDistance(adjV, choiceP3[len(choiceP3) - 1], WN1)     #path for Water Pitcher from Pebble 2
    pathW32 = printShortestDistance(adjV, choiceP3[len(choiceP3) - 1], WN2)

    if (len(pathW31) < len(pathW32)):
        print("We choose path W1")  # deciding which of the two nodes for water pitcher is closer
        choiceW3 = pathW31
    elif (len(pathW32) < len(pathW31)):
        print("We choose path W2")
        choiceW3 = pathW32
    else:
        print("Both distances are same")
        choiceW3 = pathW31

    axisP = pathToAxis(choice, edge_Axis)
    axisPathPebble = pathToAxis(choice, edge_Axis)          #changing path for start point to pebble in terms of axis
    print("AxisP:", axisPathPebble)

    axisPathPebble2 = pathToAxis(choiceP2, edge_Axis)  # changing path for start point to pebble in terms of axis
    print("AxisP2:", axisPathPebble2)

    axisPathPebble3 = pathToAxis(choiceP3, edge_Axis)  # changing path for start point to pebble in terms of axis
    print("AxisP3:", axisPathPebble3)

    axisPathWaterpi = pathToAxis(choiceW, edge_Axis)        #changing path for pebble 1 to water pitcher in terms of axis
    print("AxisW:", axisPathWaterpi)

    axisPathWaterpi2 = pathToAxis(choiceW2, edge_Axis)  # changing path for pebble 1 to water pitcher in terms of axis
    print("AxisW:", axisPathWaterpi2)

    axisPathWaterpi3 = pathToAxis(choiceW3, edge_Axis)  # changing path for pebble 1 to water pitcher in terms of axis
    print("AxisW:", axisPathWaterpi3)

    #Calculating the path instructions
    # for source to Pebble 1 to water pitcher
    PebblePath1 = axisToIns(axisP, axisPathWaterpi, 2, axis, axisWater,0,1)

    # for Pebble 1 to Pebble 2 to water pitcher
    pebblePath2 = axisToIns(axisPathPebble2, axisPathWaterpi2, axisWater, axis_Pebble2, axisWater,0,0)

    # for Pebble 2 to Pebble 3 to water pitcher
    pebblePath3 = axisToIns(axisPathPebble3, axisPathWaterpi3, axisWater, axis_Pebble3, axisWater,1,0)
    Path = PebblePath1 + pebblePath2 + pebblePath3



    return Path


"""
Function Name : background()
Input: none
Output: none
Purpose: Does a number of jobs:
        1. defines the graph by putting values into functions like add_edge, add_edge_to_cell etc
        2. has the python arena configuration dictionary which decides the different destination node
        3. converts the values in the dictionary to usable format and using functions like cellTNode, printShortestDistance, pathToAxis and axisToIns
           it creates a list of instructions to be sent to the bot
        4. opens serial port for communication bertween XBees
        5. Sends the instructions to the bot
        6. Recieves triggers for animation after pickups and drops and assigns values to flag and flaga for further processing
        
        
"""


def background():
    global flag, flaga,flagb,flagc, P1,P2,P3,W, START, Robot_start
    global cell,adjV,edge_Axis, PebbleAR1, PebbleAR2,PebbleAR3

#####################The graph is defined below this################


    for i in range(55):
        for j in range(55):
            edge_Axis[i].append(-1)

    for j in range(1, 48):
        add_edge(adjV, j, j + 1)

    add_edge(adjV, 1, 30);
    add_edge(adjV, 31, 48);
    add_edge(adjV, 49, 54);

    for j in range(49, 54):
        add_edge(adjV, j, j + 1)

    add_edge(adjV, 2, 33);
    add_edge(adjV, 5, 34);
    add_edge(adjV, 7, 36);
    add_edge(adjV, 10, 37);
    add_edge(adjV, 12, 39);
    add_edge(adjV, 15, 40);
    add_edge(adjV, 17, 42);
    add_edge(adjV, 20, 43);
    add_edge(adjV, 22, 45);
    add_edge(adjV, 25, 46);
    add_edge(adjV, 27, 48);
    # add_edge(adjV, 30, 31);
    add_edge(adjV, 32, 49);
    add_edge(adjV, 35, 50);
    add_edge(adjV, 38, 51);
    add_edge(adjV, 41, 52);
    add_edge(adjV, 44, 53);
    add_edge(adjV, 47, 54);
    #######################


    add_edge_to_cell(cell, 1, 36, 36, 7, 10, 37, 37);
    add_edge_to_cell(cell, 2, 7, 34, 5, 36, 35, 35);
    add_edge_to_cell(cell, 3, 37, 50, 35, 38, 36, 51);
    add_edge_to_cell(cell, 4, 38, 38, 37, 12, 10, 39);
    add_edge_to_cell(cell, 5, 2, 5, 34, 34, 33, 33);
    add_edge_to_cell(cell, 6, 35, 32, 33, 50, 34, 49);
    add_edge_to_cell(cell, 7, 51, 54, 49, 52, 50, 53);
    add_edge_to_cell(cell, 8, 39, 52, 51, 40, 38, 41);
    add_edge_to_cell(cell, 9, 40, 40, 39, 39, 12, 15);
    add_edge_to_cell(cell, 10, 33, 30, 32, 32, 2, 31);
    add_edge_to_cell(cell, 11, 49, 48, 31, 54, 32, 47);
    add_edge_to_cell(cell, 12, 53, 46, 47, 44, 54, 45);
    add_edge_to_cell(cell, 13, 41, 44, 53, 42, 52, 43);
    add_edge_to_cell(cell, 14, 15, 42, 16, 41, 40, 17);
    add_edge_to_cell(cell, 15, 31, 31, 48, 48, 30, 27);
    add_edge_to_cell(cell, 16, 47, 47, 27, 46, 48, 25);
    add_edge_to_cell(cell, 17, 45, 45, 25, 22, 46, 46);
    add_edge_to_cell(cell, 18, 43, 22, 45, 20, 44, 44);
    add_edge_to_cell(cell, 19, 17, 20, 43, 43, 42, 42);

    edgeAxis(edge_Axis, 1, 2, 1)
    edgeAxis(edge_Axis, 3, 4, 1)
    edgeAxis(edge_Axis, 5, 6, 1)
    edgeAxis(edge_Axis, 7, 8, 1)
    edgeAxis(edge_Axis, 30, 29, 1)
    edgeAxis(edge_Axis, 14, 15, 1)
    edgeAxis(edge_Axis, 16, 17, 1)
    edgeAxis(edge_Axis, 18, 19, 1)
    edgeAxis(edge_Axis, 20, 21, 1)
    edgeAxis(edge_Axis, 22, 23, 1)
    edgeAxis(edge_Axis, 27, 48, 1)
    edgeAxis(edge_Axis, 32, 31, 1)
    edgeAxis(edge_Axis, 33, 34, 1)
    edgeAxis(edge_Axis, 35, 36, 1)
    edgeAxis(edge_Axis, 37, 10, 1)
    edgeAxis(edge_Axis, 12, 39, 1)
    edgeAxis(edge_Axis, 40, 41, 1)
    edgeAxis(edge_Axis, 53, 52, 1)
    edgeAxis(edge_Axis, 38, 51, 1)
    edgeAxis(edge_Axis, 43, 42, 1)
    edgeAxis(edge_Axis, 45, 44, 1)
    edgeAxis(edge_Axis, 49, 50, 1)
    edgeAxis(edge_Axis, 47, 54, 1)
    edgeAxis(edge_Axis, 25, 46, 1)

    edgeAxis(edge_Axis, 4, 5, 2)
    edgeAxis(edge_Axis, 6, 7, 2)
    edgeAxis(edge_Axis, 8, 9, 2)
    edgeAxis(edge_Axis, 10, 11, 2)
    edgeAxis(edge_Axis, 12, 13, 2)
    edgeAxis(edge_Axis, 28, 27, 2)
    edgeAxis(edge_Axis, 26, 25, 2)
    edgeAxis(edge_Axis, 24, 23, 2)
    edgeAxis(edge_Axis, 20, 19, 2)
    edgeAxis(edge_Axis, 22, 21, 2)
    edgeAxis(edge_Axis, 30, 31, 2)
    edgeAxis(edge_Axis, 42, 47, 2)
    edgeAxis(edge_Axis, 46, 45, 2)
    edgeAxis(edge_Axis, 44, 43, 2)
    edgeAxis(edge_Axis, 40, 15, 2)
    edgeAxis(edge_Axis, 42, 17, 2)
    edgeAxis(edge_Axis, 38, 39, 2)
    edgeAxis(edge_Axis, 36, 37, 2)
    edgeAxis(edge_Axis, 34, 35, 2)
    edgeAxis(edge_Axis, 2, 33, 2)
    edgeAxis(edge_Axis, 50, 51, 2)
    edgeAxis(edge_Axis, 49, 32, 2)
    edgeAxis(edge_Axis, 54, 53, 2)
    edgeAxis(edge_Axis, 41, 52, 2)
    edgeAxis(edge_Axis, 47, 48, 2)

    edgeAxis(edge_Axis, 15, 16, 3)
    edgeAxis(edge_Axis, 13, 14, 3)
    edgeAxis(edge_Axis, 11, 12, 3)
    edgeAxis(edge_Axis, 10, 9, 3)
    edgeAxis(edge_Axis, 18, 17, 3)
    edgeAxis(edge_Axis, 30, 1, 3)
    edgeAxis(edge_Axis, 2, 3, 3)
    edgeAxis(edge_Axis, 5, 34, 3)
    edgeAxis(edge_Axis, 28, 29, 3)
    edgeAxis(edge_Axis, 26, 27, 3)
    edgeAxis(edge_Axis, 24, 25, 3)
    edgeAxis(edge_Axis, 22, 45, 3)
    edgeAxis(edge_Axis, 20, 43, 3)
    edgeAxis(edge_Axis, 53, 44, 3)
    edgeAxis(edge_Axis, 42, 41, 3)
    edgeAxis(edge_Axis, 40, 39, 3)
    edgeAxis(edge_Axis, 38, 37, 3)
    edgeAxis(edge_Axis, 32, 33, 3)
    edgeAxis(edge_Axis, 48, 31, 3)
    edgeAxis(edge_Axis, 50, 35, 3)
    edgeAxis(edge_Axis, 36, 7, 3)
    edgeAxis(edge_Axis, 52, 51, 3)
    edgeAxis(edge_Axis, 49, 54, 3)
    edgeAxis(edge_Axis, 47, 46, 3)
    # print(edge_Axis)


################################The graph is defined above this############


    arena_config =  {0: ("Water Pitcher", 7, "3-3"), 1: ("Pebble",  14, "1-1"), 3: ("Pebble",   1 , "2-2"),
                    5: ("Pebble", 15, "3-3")}                                                                  #using only the first two dictionary keys for progress task
    Robot_start = "START - 1"
    if (Robot_start == "START - 1"):
        START = 16
    else:
        START = 1





    W1,P1,P2,P3 = arena_config.keys()      #Storing the Aruco ID of each object in it's corresponding variable for ID
    print(W1,P1,P2,P3)


    ####DISCLAIMER###########
    """
    We had started defining the graph and working with the path planning right during task 3 and we were using
    the image of the graph given in the rulebook with different cells numbered and we followed the axes given 
    in it. Everything worked fine if we used that exact graph as a reference. But a couple of days before the 
    submission of the progress task we realized the axes in the actual arena are different:
    
    Axis 1-1 on the arena was axis 2-2 on the numbered graph
    Axis 2-2 on the arena was axis 3-3 on the numbe
    red graph
    Axis 3-3 on the arena was axis 1-1 on the numbered graph
    
    Since there wasn't enough time to correct everything and it would've been error prone to completely change
    the graph we simply use the following if else statements to convert the arena configurations to the
    configurations of our graph we had defined and everything worked fine again.

    """
    startaxis = 2                       #the axis at either starting point is always 2
    START = 16

    ##################FIND ALL POSSIBLE PATHS BETWEEN THE PEBBLES AND WATER PITCHER#############
    #####(AS IN CHANGE THE ORDER OF THE PEBBLES TO BE TRAVERSED)##############
    Path1 = calculatePath(arena_config,W1,P1,P2,P3, Robot_start)            #First go to pebble P1, then P2 and then P3
    Path2 = calculatePath(arena_config,W1,P2,P1,P3, Robot_start)            #First go to pebble P2, then P1 and then P3
    Path3 = calculatePath(arena_config,W1,P2,P3,P1, Robot_start)            #First go to pebble P2, then P3 and then P1
    Path4 = calculatePath(arena_config,W1,P1,P3,P2, Robot_start)            #First go to pebble P1, then P3 and then P2
    Path5 = calculatePath(arena_config,W1,P3,P2,P1, Robot_start)            #First go to pebble P3, then P2 and then P1
    Path6 = calculatePath(arena_config,W1,P3,P1,P2, Robot_start)            #First go to pebble P3, then P1 and then P2

    arrayPath = [Path1,Path2,Path3,Path4,Path5,Path6]
    arrayLengthPath = [len(Path1),len(Path2),len(Path3),len(Path4),len(Path5),len(Path6)]

    print("Path1:",Path1)
    print("Path2:",Path2)
    print("Path3:",Path3)
    print("Path4:",Path4)
    print("Path5:",Path5)
    print("Path6:",Path6)



    ###############CHECK WHICH PATH IS SHORTEST AND FOLLOW THAT#####################
    finalPathToBot = arrayPath[arrayLengthPath.index(min(arrayLengthPath))]
    #FINAL INSTRUCTION#
    print(finalPathToBot)

    if(finalPathToBot == Path1):
        PebbleAR1,PebbleAR2,PebbleAR3 = P1,P2,P3
    elif(finalPathToBot == Path2):
        PebbleAR1,PebbleAR2,PebbleAR3 = P2,P1,P3
    elif (finalPathToBot == Path3):
        PebbleAR1, PebbleAR2, PebbleAR3 = P1, P3, P1
    elif (finalPathToBot == Path4):
        PebbleAR1, PebbleAR2, PebbleAR3 = P1, P3, P2
    elif (finalPathToBot == Path5):
        PebbleAR1, PebbleAR2, PebbleAR3 = P3, P2, P1
    elif (finalPathToBot == Path6):
        PebbleAR1, PebbleAR2, PebbleAR3 = P3, P1, P2

    ###################################XBee communication###########################################
    ser = serial.Serial("COM4", 9600, timeout=0.002)        # COM4 was used on our device


    if (ser.isOpen()):
        while True:
            a = ser.read(1)
            print(a)
            if (a == b'w'):
                break
    while True:
        if (ser.isOpen()):  # Checking if input port is open ie capable of communication


            for i in range(len(finalPathToBot)):
                ser.write(finalPathToBot[i].encode())  # Writing binary data onto the serial port
                #time.sleep(0.03)  # letting the atmega recieve the character and send it back

                rec = ser.read(1)  # recieving characters from atmega serially
                print(rec)
                if rec == b's':
                    flaga = 'p'
                    print("PICKUP #1")
                if rec == b'd':
                    flag = 'd'
                    print("DROP #1")
                if rec == b'g':
                    flagb = 'p'
                    print("PICKUP #2")
                if rec == b'h':
                    flagc = 'p'
                    print("PICKUP #3")
                if rec == b'c':
                    flag = 'c'
                    print("DROP #2")
                if rec == b'f':
                    flag = 'f'
                    print("DROP #3")


"""
Function Name : init_gl()
Input: None
Output: None
Purpose: Initialises various parameters related to OpenGL scene.
"""


def init_gl():
    global texture_object, texture_background
    global crow, pebble, pebble_dim, waterE, waterF, waterM1, waterM2

    waterE = OBJ('pitcher.obj', swapyz=True)                #obj for the Water pitcher (Empty)

    waterM1 = OBJ('pitcherMiddle1.obj', swapyz=True)        #obj for the Water pitcher (Medium 1)
    waterM2 = OBJ('pitcher3.obj', swapyz=True)              #obj for the Water pitcher (Medium 2)

    waterF = OBJ('pitcherFilled.obj', swapyz=True)          #obj for the Water pitcher (Filled)


    pebble = OBJ('finalrock.obj', swapyz=True)              #obj for the initial pebble stack
    pebble_dim = OBJ('PebbleDim.obj', swapyz=True)#obj for the diminished pebble stack

     #the obj files for the Crow animation is loaded along with the emoji that represents its current emotions#

    #The if statement simply loads one of the two versions of the crow models we have depending on the Start position.
    #This is purely for aesthetics purposes since we realized that the same model didn't look good on both sides
    #Since the emoji part is supposed to stay beyond the crow, we simple made two models, each being the mirror image
    #of each other on the Y - Axis. Transforming the image to flip it in code was not consistent so we made two models instead.
    #This doesn't take extra time since only one model is loaded, either the one at start-1 or at start-2

    if Robot_start == "START - 1":
        load('crowEmoji', frames)  # Crow animation flying to the pebble

        load('bend', frames_bend)  # Crow animation with bent neck

        load('crowEmojiPebble', frames_crowrock)  # Crow animation flying the pebble back to the water pitcher

    elif Robot_start == "START - 2":

        load('crowEmoji2',frames)                    #Crow animation flying to the pebble

        load('Bend2',frames_bend)                    #Crow animation with bent neck

        load('crowEmojiPebble2',frames_crowrock)     #Crow animation flying the pebble back to the water pitcher





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
                # if i[0] == 0:
                #     overlay(frame, ar_list, i[0], "texture_1.png")
                # if i[0] == 2:
                #     overlay(frame, ar_list, i[0], "texture_2.png")
                # if i[0] == 1:
                #     overlay(frame, ar_list, i[0], "texture_3.png")
                # if i[0] == 610:
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
    markerLength = 0.127
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
    i = 60.0
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
    glLoadIdentity()
    glPushMatrix()
    glTranslatef(0.0, 0.0, 10.0)
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)

    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST)



    glTexImage2D(GL_TEXTURE_2D, 0, 3, ix, iy, 0, GL_RGBA, GL_UNSIGNED_BYTE, bg_image)
    glPopMatrix()

    return None


"""
Function Name : load()
Input: directory(string, name of the folder in which the obj files are), frames(a list where the files will be appended)
Output: None
Purpose: Takes the directory name as input and stores all the obj files in it inside the list frames. 
        Which will be used for animating the crow later
"""


def load(directory,frames):
    os.chdir(directory)

    for file in glob.glob("*.obj"):
        frames.append(OBJ(file,swapyz=True))

    os.chdir('..')
    frames_length = len(frames)


"""
Function Name : next_frame()
Input: frames(a list where the obj files for each frame of animation are stored)
Output: frames[i].gl_list
Purpose: Takes the frame list as input and returns the frames one by one to display animation on the openGL window
"""

def next_frame(frames):
    global i
    i += 1

    #if frame_index >= self.frames_length:
    if i >= len(frames):
        i = 0

    return frames[i].gl_list




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
    global flag, bend, CrowPick, CrowBeakClose
    for x in ar_list:
        if ar_id == x[0]:
            centre, rvec, tvecs = x[1], x[2], x[3]

    rmtx = cv2.Rodrigues(rvec)[0]
    offset = [[[-0.127*19/8, -0.127*2.8, 0]]]
    tvecs = tvecs - offset
    font = cv2.FONT_HERSHEY_SIMPLEX #font for displaying text (below)

    cv2.putText(img, "Id: " + str(ar_id), centre, font, 1, (0,255,0),2,cv2.LINE_AA)


    view_matrix = np.array([[rmtx[0][0], rmtx[0][1], rmtx[0][2],  (tvecs[0][0][0])*12.5],
                            [rmtx[1][0], rmtx[1][1], rmtx[1][2], (tvecs[0][0][1]+0.16)*11],
                            [rmtx[2][0], rmtx[2][1], rmtx[2][2], tvecs[0][0][2]*8],
                            [0.0, 0.0, 0.0, 1.0]])


    view_matrix = view_matrix * INVERSE_MATRIX
    view_matrix = np.transpose(view_matrix)

    glPushMatrix()
    glLoadMatrixd(view_matrix)
    print(PebbleAR1,PebbleAR2,PebbleAR3)
    ##########Asigning different Models to different arucos##########################



    if ar_id == 0:                               #AR ID for water pitcher is always 0


        if flag == 0:
            glCallList(waterE.gl_list)
        elif flag == 'd':
            glCallList(waterM1.gl_list)
            CrowPick = 0
            CrowBeakClose = 1
        elif flag == 'c':
            glCallList(waterM2.gl_list)
            CrowPick = 0
            CrowBeakClose = 1
        elif flag == 'f':
            glCallList(waterF.gl_list)
            bend = 1

    if (ar_id) == PebbleAR1:                          #AR ID for pebble  1
        glScale(2.8, 2.8,2.8)
        if flaga == 0:
            glCallList(pebble.gl_list)
        elif flaga == 'p':
            glCallList(pebble_dim.gl_list)
            CrowPick = 1
            CrowBeakClose = 0

    if ar_id == PebbleAR2:                           #AR ID for Pebble 2
        glScale(2.8, 2.8,2.8)
        if flagb == 0:
            glCallList(pebble.gl_list)
        elif flagb == 'p':
            glCallList(pebble_dim.gl_list)
            CrowPick = 1
            CrowBeakClose = 0

    if ar_id == PebbleAR3:                               #AR ID for Pebble 3
        glScale(2.8, 2.8,2.8)
        if flagc == 0:
            glCallList(pebble.gl_list)
            print("FLAGC",flagc)
        elif flagc == 'p':
            glCallList(pebble_dim.gl_list)
            CrowPick = 1
            CrowBeakClose = 0
            print("FLAGC",flagc)

    if str(ar_id) == '10':                              #AR ID for crow is always 10
        glScale(1.25,1.25,1.25)
        if(bend ==1):
            glCallList(next_frame(frames_bend))
        elif(flaga == 'p' and flag == 0) or (flagb == 'p' and flag == 'd') or (flagc == 'p' and flag == 'c'):
            glCallList(next_frame(frames_crowrock))
        elif(flag == 'd' and flagb == 0) or (flag == 'c' and flagc ==0) or (flag == 0 and flaga == 0):
            glCallList(next_frame(frames))



    #################################################3


    glPopMatrix()


########################################################################

if __name__ == "__main__":
    main()
