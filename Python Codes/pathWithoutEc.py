
import sys
def add_edge(dict, V1, V2):
    dict[V1].append(V2)
    dict[V2].append(V1)
    #print("edge between: ", V1, V2)

def add_edge_to_cell(dict, cell, node11, node12, node21, node22, node31, node32):
    dict[cell].append(node11)
    dict[cell].append(node12)
    dict[cell].append(node21)
    dict[cell].append(node22)
    dict[cell].append(node31)
    dict[cell].append(node32)

def cellToNode(dict, cellNo, axis):
    if (axis == 1):
        # // cout << "AXIS = 1:  \t" << a[cellNo][0] << "\t";
        # // cout << a[cellNo][1] << "\n";
        N1 = dict[cellNo][0]
        N2 = dict[cellNo][1]
    elif(axis==2):
        N1 = dict[cellNo][2]
        N2 = dict[cellNo][3]
    elif(axis==3):
        N1 = dict[cellNo][4]
        N2 = dict[cellNo][5]

    return N1,N2

def edgeAxis(dictE_A, V1, V2, axis):
    dictE_A[V1][V2] = axis
    # print("axis between:", V1, V2, axis)
    dictE_A[V2][V1] = axis


def printShortestDistance(adjV, s, dest):

    pred = []
    dist = []
    path = []
    pred = BFS(adjV,s,dest)
    # print(pred)
    crawl = dest
    path.append(crawl)
    #print(pred)
    while(pred[crawl]!= -1):
        path.append(pred[crawl])
        crawl = pred[crawl]


    path.reverse()
    print("path is: ", path)
    return path



def BFS(adjV, src, dest):
    qu = []
    visited = [False]*55
    dist = [sys.maxsize]*55
    pred = [-1]*55
    # for i in range(1,55):
    #     dist[i] = sys.maxsize
    #     pred[i] = -1


    visited[src] = True
    dist[src] = 0
    qu.append(src)

    while(qu):
        u = qu.pop(0)
        for i in range(len(adjV[u])):
            if(visited[adjV[u][i]]==False):
                visited[adjV[u][i]]=True
                dist[adjV[u][i]] = dist[u]+1
                pred[adjV[u][i]] = u
                qu.append(adjV[u][i])

                if(adjV[u][i]==dest):
                    return pred



def pathToAxis(path,edge_Axis):
    pathinAxis = []
    for i in range(len(path)-1):
        # print(len(path), path[i], path[i+1])
        pathinAxis.append(edge_Axis[path[i]][path[i+1]])
    return pathinAxis
#############################################################3
adjV = {}
for i in range(1,55):
    adjV[i] = []
cell = {}
for i in range(1,20):
    cell[i] = []
edge_Axis = {}
for i in range(55):
    edge_Axis[i] = []

for i in range(55):
    for j in range(55):
        edge_Axis[i].append(-1)



for j in range(1,48):
    add_edge(adjV, j, j + 1)

add_edge(adjV, 1, 30);
add_edge(adjV, 31, 48);
add_edge(adjV, 49, 54);

for j in range(49,54):
    add_edge(adjV,j,j+1)

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
        #add_edge(adjV, 30, 31);
add_edge(adjV, 32, 49);
add_edge(adjV, 35, 50);
add_edge(adjV, 38, 51);
add_edge(adjV, 41, 52);
add_edge(adjV, 44, 53);
add_edge(adjV, 47, 54);
#######################


add_edge_to_cell(cell, 1, 9, 36, 7, 10, 8, 37);
add_edge_to_cell(cell, 2, 7, 34, 5, 36, 6, 35);
add_edge_to_cell(cell, 3, 37, 50, 35, 38, 36, 51);
add_edge_to_cell(cell, 4, 11, 38, 37, 12, 10, 39);
add_edge_to_cell(cell, 5, 2, 5, 3, 34, 4, 33);
add_edge_to_cell(cell, 6, 35, 32, 33, 50, 34, 49);
add_edge_to_cell(cell, 7, 51, 54, 49, 52, 50, 53);
add_edge_to_cell(cell, 8, 39, 52, 51, 40, 38, 41);
add_edge_to_cell(cell, 9, 13, 40, 39, 14, 12, 15);
add_edge_to_cell(cell, 10, 33, 30, 1, 32, 2, 31);
add_edge_to_cell(cell, 11, 49, 48, 31, 54, 32, 47);
add_edge_to_cell(cell, 12, 53, 46, 47, 44, 54, 45);
add_edge_to_cell(cell, 13, 41, 44, 53, 42, 52, 43);
add_edge_to_cell(cell, 14, 15, 42, 41, 16, 40, 17);
add_edge_to_cell(cell, 15, 31, 28, 29, 48, 30, 27);
add_edge_to_cell(cell, 16, 47, 26, 27, 46, 48, 25);
add_edge_to_cell(cell, 17, 45, 24, 25, 22, 46, 23);
add_edge_to_cell(cell, 18, 43, 22, 45, 20, 44, 21);
add_edge_to_cell(cell, 19, 17, 20, 43, 18, 42, 19);

edgeAxis(edge_Axis, 1,2,1)
edgeAxis(edge_Axis, 3,4,1)
edgeAxis(edge_Axis, 5,6,1)
edgeAxis(edge_Axis, 7,8,1)
edgeAxis(edge_Axis, 30,29,1)
edgeAxis(edge_Axis, 14,15,1)
edgeAxis(edge_Axis, 16,17,1)
edgeAxis(edge_Axis, 18,19,1)
edgeAxis(edge_Axis, 20,21,1)
edgeAxis(edge_Axis, 22,23,1)
edgeAxis(edge_Axis, 27,48,1)
edgeAxis(edge_Axis, 32,31,1)
edgeAxis(edge_Axis, 33,34,1)
edgeAxis(edge_Axis, 35,36,1)
edgeAxis(edge_Axis, 37,10,1)
edgeAxis(edge_Axis, 12,39,1)
edgeAxis(edge_Axis, 40,41,1)
edgeAxis(edge_Axis, 53,52,1)
edgeAxis(edge_Axis, 38,51,1)
edgeAxis(edge_Axis, 43,42,1)
edgeAxis(edge_Axis, 45,44,1)
edgeAxis(edge_Axis, 49,50,1)
edgeAxis(edge_Axis, 47,54,1)
edgeAxis(edge_Axis, 25,46,1)


edgeAxis(edge_Axis, 4,5,2)
edgeAxis(edge_Axis, 6,7,2)
edgeAxis(edge_Axis, 8,9,2)
edgeAxis(edge_Axis, 10,11,2)
edgeAxis(edge_Axis, 12,13,2)
edgeAxis(edge_Axis, 28,27,2)
edgeAxis(edge_Axis, 26,25,2)
edgeAxis(edge_Axis, 24,23,2)
edgeAxis(edge_Axis, 20,19,2)
edgeAxis(edge_Axis, 22,21,2)
edgeAxis(edge_Axis, 30,31,2)
edgeAxis(edge_Axis, 42,47,2)
edgeAxis(edge_Axis, 46,45,2)
edgeAxis(edge_Axis, 44,43,2)
edgeAxis(edge_Axis, 40,15,2)
edgeAxis(edge_Axis, 42,17,2)
edgeAxis(edge_Axis, 38,39,2)
edgeAxis(edge_Axis, 36,37,2)
edgeAxis(edge_Axis, 34,35,2)
edgeAxis(edge_Axis, 2,33,2)
edgeAxis(edge_Axis, 50,51,2)
edgeAxis(edge_Axis, 49,32,2)
edgeAxis(edge_Axis, 54,53,2)
edgeAxis(edge_Axis, 41,52,2)
edgeAxis(edge_Axis, 48,47,2)


edgeAxis(edge_Axis, 15,16,3)
edgeAxis(edge_Axis, 13,14,3)
edgeAxis(edge_Axis, 11,12,3)
edgeAxis(edge_Axis, 10,9,3)
edgeAxis(edge_Axis, 18,17,3)
edgeAxis(edge_Axis, 30,1,3)
edgeAxis(edge_Axis, 2,3,3)
edgeAxis(edge_Axis, 5,34,3)
edgeAxis(edge_Axis, 28,29,3)
edgeAxis(edge_Axis, 26,27,3)
edgeAxis(edge_Axis, 24,25,3)
edgeAxis(edge_Axis, 22,45,3)
edgeAxis(edge_Axis, 20,43,3)
edgeAxis(edge_Axis, 53,44,3)
edgeAxis(edge_Axis, 42,41,3)
edgeAxis(edge_Axis, 40,39,3)
edgeAxis(edge_Axis, 38,37,3)
edgeAxis(edge_Axis, 32,33,3)
edgeAxis(edge_Axis, 48,31,3)
edgeAxis(edge_Axis, 50,35,3)
edgeAxis(edge_Axis, 36,7,3)
edgeAxis(edge_Axis, 52,51,3)
edgeAxis(edge_Axis, 49,54,3)
edgeAxis(edge_Axis, 47,46,3)
#print(edge_Axis)






startaxis = 2
axis = 1
axisWater = 2
print("Axis Pebble :", axis, "Axis WatP:",axisWater)
cellNo = 16
water =  8
source = 1
PN1,PN2 = cellToNode(cell,cellNo,axis)
WN1,WN2 = cellToNode(cell, water, axisWater)
pathN1=printShortestDistance(adjV,source,PN1)
# print("Distance N1:", len(pathN1))
# print("Axis N1:", pathToAxis(pathN1,edge_Axis))

pathN2=printShortestDistance(adjV,source,PN2)

if(len(pathN1)<len(pathN2)):
    print("We choose path N1")
    choice = pathN1
elif(len(pathN2)<len(pathN1)):
    print("We choose path N2")
    choice = pathN2
else:
    print("Both distances are same")
    choice = pathN1
    # if (pathToAxis(pathN1) == axis):
    #     print("choosing N1")
    # else:
    #     print("choosing N2")

pathW1 = printShortestDistance(adjV, choice[len(choice)-1],WN1)
pathW2 = printShortestDistance(adjV, choice[len(choice)-1],WN2)

if(len(pathW1)<len(pathW2)):
    print("We choose path W1")
    choiceW = pathW1
elif(len(pathW2)<len(pathW1)):
    print("We choose path W2")
    choiceW = pathW2
else:
    print("Both distances are same")
    choiceW = pathN1

print("DistanceP:", len(choice))
axisP = pathToAxis(choice,edge_Axis)
axisPathPebble = pathToAxis(choice,edge_Axis)
print("AxisP:", axisPathPebble)

axisPathWaterpi = pathToAxis(choiceW,edge_Axis)
print("AxisW:", axisPathWaterpi)

def axisToIns(axisIns,axisW, startAxis,axisP,axisWater):
    ins = []
    if(axisIns[0]== 3):
        ins.append('r')
    elif(axisIns[0]==1):
        ins.append('l')

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

        if(i == len(axisIns)-1):
            if(axisIns[i]==axisP):
                ins.append('s')
            elif(axisIns[i]==2 and axisP==1):
                ins.append('q')                 #right by 60 degrees
            elif(axisIns[i]==1 and axisP == 2):
                ins.append('q')                 #right by 60 degrees
            elif(axisIns[i]==3 and axisP==2):
                ins.append('w')                 #left by 60 degrees
            elif (axisIns[i] == 3 and axisP == 2):
                ins.append('w')
            elif (axisIns[i] == 1 and axisP == 3):
                ins.append('q')
            elif (axisIns[i] == 3and axisP == 1):
                ins.append('w')

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

        if(i == len(axisPathWaterpi)-1):
            if(axisPathWaterpi[i]==axisWater):
                ins.append('d')
            elif (axisPathWaterpi[i] == 2 and axisWater == 1):
                ins.append('q')  # right by 60 degrees
            elif (axisPathWaterpi[i] == 1 and axisWater == 2):
                ins.append('q')  # right by 60 degrees
            elif (axisPathWaterpi[i] == 3 and axisWater == 2):
                ins.append('w')  # left by 60 degrees
            elif (axisPathWaterpi[i] == 3 and axisWater == 2):
                ins.append('w')
            elif (axisPathWaterpi[i] == 1 and axisWater == 3):
                ins.append('q')
            elif (axisPathWaterpi[i] == 3 and axisWater == 1):
                ins.append('w')
    print(ins)
    return ins


pebbleWay = axisToIns(axisP,axisPathWaterpi,startaxis,axis,axisWater)



