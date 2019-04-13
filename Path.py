
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

def printShortestDistance(adjV, s, dest):

    pred = []
    dist = []
    path = []
    pred = BFS(adjV,s,dest)
    print(pred)
    crawl = dest
    path.append(crawl)
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

    return False


#############################################################3
adjV = {}
for i in range(1,55):
    adjV[i] = []
cell = {}
for i in range(1,20):
    cell[i] = []



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

axis = 2
cellNo = 19
source = 34
N1,N2 = cellToNode(cell,cellNo,axis)
print(N1,N2)
pathN1=printShortestDistance(adjV,source,N1)
print("Distance N1:", len(pathN1))
pathN2=printShortestDistance(adjV,source,N2)
print("Distance N2:", len(pathN2))

