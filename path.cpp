
// CPP code for printing shortest path between 
// two vertices of unweighted graph 
#include <bits/stdc++.h> 
using namespace std; 
  
// utility function to form edge between two vertices 
// source and dest 
void add_edge(vector<int> adj[], int src, int dest) 
{ 
    adj[src].push_back(dest); 
    adj[dest].push_back(src); 
} 
  
// a modified version of BFS that stores predecessor 
// of each vertex in array p 
// and its distance from source in array d 
bool BFS(vector<int> adj[], int src, int dest, int v, 
                            int pred[], int dist[]) 
{ 
    // a queue to maintain queue of vertices whose 
    // adjacency list is to be scanned as per normal 
    // DFS algorithm 
    list<int> queue; 
  
    // boolean array visited[] which stores the 
    // information whether ith vertex is reached 
    // at least once in the Breadth first search 
    bool visited[v]; 
  
    // initially all vertices are unvisited 
    // so v[i] for all i is false 
    // and as no path is yet constructed 
    // dist[i] for all i set to infinity 
    for (int i = 0; i < v; i++) { 
        visited[i] = false; 
        dist[i] = INT_MAX; 
        pred[i] = -1; 
    } 
  
    // now source is first to be visited and 
    // distance from source to itself should be 0 
    visited[src] = true; 
    dist[src] = 0; 
    queue.push_back(src); 
  
    // standard BFS algorithm 
    while (!queue.empty()) { 
        int u = queue.front(); 
        queue.pop_front(); 
        for (int i = 0; i < adj[u].size(); i++) { 
            if (visited[adj[u][i]] == false) { 
                visited[adj[u][i]] = true; 
                dist[adj[u][i]] = dist[u] + 1; 
                pred[adj[u][i]] = u; 
                queue.push_back(adj[u][i]); 
  
                // We stop BFS when we find 
                // destination. 
                if (adj[u][i] == dest) 
                   return true; 
            } 
        } 
    } 
  
    return false; 
} 
  
// utility function to print the shortest distance  
// between source vertex and destination vertex 
void printShortestDistance(vector<int> adj[], int s,  
                                    int dest, int v) 
{ 
    // predecessor[i] array stores predecessor of 
    // i and distance array stores distance of i 
    // from s 
    int pred[v], dist[v]; 
  
    if (BFS(adj, s, dest, v, pred, dist) == false) 
    { 
        cout << "Given source and destination"
             << " are not connected"; 
        return; 
    } 
  
    // vector path stores the shortest path 
    vector<int> path; 
    int crawl = dest; 
    path.push_back(crawl); 
    while (pred[crawl] != -1) { 
        path.push_back(pred[crawl]); 
        crawl = pred[crawl]; 
    } 
  
    // distance from source is in distance array 
    cout << "Shortest path length is : "
        << dist[dest]; 
  
    // printing path from source to destination 
    cout << "\nPath is::\n"; 
    for (int i = path.size() - 1; i >= 0; i--) 
        cout << path[i] << " "; 
} 
  
// Driver program to test above functions 
int main() 
{ 
    // no. of vertices 
    int v = 55;  
  
    // array of vectors is used to store the graph 
    // in the form of an adjacency list 
    vector<int> adj[v]; 
  
    // Creating graph given in the above diagram. 
    // add_edge function takes adjacency list, source  
    // and destination vertex as argument and forms 
    // an edge between them. 
    // add_edge(adj, 0, 1); 
    // add_edge(adj, 0, 3); 
    add_edge(adj, 1, 2); 
    for(int j =1; j<48; j++){
        add_edge(adj, j, j+1); 
        cout<<"edge between \t"<<j<<"\t"<<j+1<<"\n";
    }
        add_edge(adj, 1, 30); 
        add_edge(adj, 31, 48); 

    for(int j =49; j<54; j++){
        add_edge(adj, j, j+1); 
        cout<<"edge between\t"<<j<<"\t"<<j+1<<"\n";
    }
        
        add_edge(adj, 2, 33); 
        add_edge(adj, 5, 34); 
        add_edge(adj, 7, 36); 
        add_edge(adj, 10, 37); 
        add_edge(adj, 12, 39); 
        add_edge(adj, 15, 40); 
        add_edge(adj, 17, 42); 
        add_edge(adj, 20, 43); 
        add_edge(adj, 22, 45); 
        add_edge(adj, 25, 46); 
        add_edge(adj, 27, 48); 
        // add_edge(adj, 30, 31); 
        add_edge(adj, 32, 49); 
        add_edge(adj, 35, 50); 
        add_edge(adj, 38, 51); 
        add_edge(adj, 41, 52); 
        add_edge(adj, 44, 53); 
        add_edge(adj, 47, 54); 









    // add_edge(adj, 3, 4); 
    // add_edge(adj, 3, 7); 
    // add_edge(adj, 4, 5); 
    // add_edge(adj, 4, 6); 
    // add_edge(adj, 4, 7); 
    // add_edge(adj, 5, 6); 
    // add_edge(adj, 6, 7); 
    int source = 30, dest = 41; 
    printShortestDistance(adj, source, dest, v); 
    return 0; 
} 
