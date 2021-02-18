#include "Graph.h"
#include <queue>
#include <stack>
#include <iostream>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <iterator>
#include <string>
#include <cmath>
//#include <boost/algorithm/string.hpp>

using namespace std;

Graph::Graph(){

}

//constructor for graph given # of vertices
Graph::Graph (int num) {
  maxVertices = num;
  vertices = 0;
  graph = new AdjacencyList[num];
  for(int i = 0; i < num; i++) {
    graph[i].head = nullptr;
    latitudes.push_back(0);
    longitudes.push_back(0);
    names.push_back("");
  }
}

//default constructor for AdjacencyListNode
Graph::AdjacencyListNode::AdjacencyListNode(){
  end = 0;
  next = nullptr;
}

//default constructor for AdjacencyList
Graph::AdjacencyList::AdjacencyList(){
  head = nullptr;
}

Graph::Edge::Edge(){
  source = 0;
  dest = 0;
  weight_ = 0.0;
  label = "XX";
}

Graph::Edge::Edge(int uno, int dos, float weight){
  cross = false;
  discovery = false;
  source = uno;
  dest = dos;
  weight_ = weight;
  label = std::to_string(uno) + std::to_string(dos);
}

//dummy function to see if makefile works
int Graph::numVertices(){
  return vertices;
}

//returns maximum possible number of vertices in graph
int Graph::getMaxVertices() {
  return maxVertices;
}

//creating new node for an adjacency list
Graph::AdjacencyListNode * Graph::newAdjacencyListNode(int n) {
  AdjacencyListNode * newNode = new AdjacencyListNode;
  newNode->end = n;
  newNode->next = nullptr;
  return newNode;
}

//adding a new edge to the graph
void Graph::insertEdge(int source, int dest, float weight) {
  AdjacencyListNode * newNode = newAdjacencyListNode(dest);
  newNode->next = graph[source].head;
  graph[source].head = newNode;
  newNode = newAdjacencyListNode(source);
  newNode->next = graph[dest].head;
  graph[dest].head = newNode;
  Edge * newEdge = new Edge(source, dest, weight);
  edges.push_back(newEdge);
}

//remove directed edge from source->destination
void Graph::removeEdge(int source, int dest) {
  //if dest vertex is head of adjacency list
  if (graph[source].head->end == dest) {
    AdjacencyListNode * next = graph[source].head->next;
    delete graph[source].head;
    graph[source].head = next;
    return;
  }

  //else loop through adjacency list
  AdjacencyListNode * curr = graph[source].head->next;
  AdjacencyListNode * prev = graph[source].head;
  while (curr != NULL) {
    if (curr->end == dest) {
      //remove from adjacency list
      prev->next = curr->next;
      delete curr;
      break;
    }
    prev = curr;
    curr = curr->next;
  }
  for(size_t i = 0; i < edges.size(); i++){
    if(edges[i]->source == source && edges[i]->dest == dest){
      edges[i] = new Edge();
      break;
    }
  }
}


void Graph::insertVertex(int n) {
  graph[n] = AdjacencyList();
  vertices++;
}


void Graph::removeVertex(int n) {
  //delete adjacency list
  graph[n] = AdjacencyList();

  //delete any edges shared with n
  for (int i = 0; i < vertices; i++) {
    AdjacencyListNode * curr = graph[i].head;
    while (curr != NULL) {
      if (curr->end == n) {
        removeEdge(n, i);
      }
      curr = curr->next;
    }
  }
}

Graph::AdjacencyList Graph::incidentEdges(int n) {
  return graph[n];
}

bool Graph::areAdjacent(int source, int dest) {
  AdjacencyListNode * curr = graph[source].head;
  while (curr != NULL) {
    if (curr->end == dest) {
      return true;
    }
    curr = curr->next;
  }
  return false;


}

Graph::Edge * Graph::getEdge(int source, int dest) {

  for (size_t i = 0; i < edges.size(); i++) {
    Edge * edge = edges[i];
    if (edge->source == source && edge->dest == dest) {
      return edges[i];
    }
  }
  return nullptr;
}

void Graph::setLatLong(int n, float latitude, float longitude) {
  latitudes[n] = latitude;
  longitudes[n] = longitude;
}

//will return distance between two vertices/airports in kilometers
float Graph::distance(int source, int dest){
  float radius = 6371;
  //pull lat and long from corresponding vectors
  float lat1 = latitudes[source];
  float long1 = longitudes[source];
  float lat2 = latitudes[dest];
  float long2 = longitudes[dest];
  //convert to radians
  lat1 *= M_PI / 180;
  long1 *= M_PI / 180;
  lat2 *= M_PI / 180;
  long2 *= M_PI / 180;
  //Haversine formula
  float dlat = lat2 - lat1;
  float dlong = long2 - long1;
  float result = pow(sin(dlat / 2), 2) + cos(lat1) * cos(lat2) * pow(sin(dlong / 2), 2);
  result = 2 * asin(sqrt(result));
  //convert to kilomters using the radius of the earth
  result *= radius;
  return result;
}

/**************/
void Graph::setName(int id, std::string setName) {

  //boost::to_upper(setName);
  // std::cout << id << setName << '\n';
  names[id] =  setName;
  // std::cout << "!!!!" << names[id] << '\n';
}


int Graph::getID(std::string name) {
  //boost::to_upper(name);
  for (size_t i = 0; i < names.size(); i++) {
    if (names[i] == name) {
      return i;
    }
  }
  return -1;
}

std::string Graph::getName(int id) {
  return names[id];
}
/*************/

/*
Setup:
  Maintain a queue
  Maintain a table of vertices with following features:
  Boolean flag - visited,  Distance from the start,  Predecessor,  List of adjacent vertices

*/

void Graph::BFS(int n) {
  bool * visited = new bool[numVertices()];
  for( int i =0; i < numVertices(); i++) {
    visited[i]=false;
  }
  std::queue<int> q;

  //Add the starting point
  q.push(n);
  visited[n] = true;

  //While the queue is not empty
  while (!q.empty()) {
    //Dequeue v
    int visit = q.front();
    q.pop();
    AdjacencyList al = incidentEdges(visit);
    // For all of the unlabeled edges adjacent to v
    AdjacencyListNode * current = al.head;
    while (current != NULL) {
      int node = current->end;
      current = current->next;

      //find corresponding edge
      Edge * edge = getEdge(visit, node);
      //if edge has not been inserted into graph
      if (edge == NULL) continue;

      // If an adjacent edge “discovers” a new vertex t:
      if (!visited[node]) {
        // Label the edge a “discovery edge”
        edge->discovery = true;
        // Enqueue t, update the information of t (distance = dist(v) + 1, predecessor = v)
        q.push(node);
        visited[node] = true;
      // If an adjacent edge is between two visited vertices
      } else {
        // Label the edge a “cross edge”
        edge->cross = true;
      }
      // current = current->next;
    }
  }
}

/* Notes on Dijkstra's Alg and Betweeness:

  Dijkstra's Algorithm will compute the shortest path between a source and an
  arbitrary destination. Betweeness Centrality requires us to store all shortest
   paths from a given source node to each node in the graph. And to have a counter at each node,
  that represents how many shortest paths travel through a given node.

  Betweeness Centrality Idea:
    Pick a source vertex
    Create a vector to store all shortest_paths
    for each vertex in the graph
        compute shortest distance from source to graph[i]
        Store this path in the Paths_vector
    for each path in the vector
        for each edge in the shortest_path[i]
          increment the count at each node (meaning one of the shortest paths visits that node)

    //THIS IS SLIGHTLY DIFFERENT THEN GENERALIZED BETWEENESS CENTRALITY.
    because we must provide a starting node? Internet is vague...
*/

//Dijkstra's Algorithm
std::vector<float> Graph::dijkstra(int source) {

    //output of each vertex distance from a source
    std::vector<float> distance_;
    distance_.resize(numVertices(), INFINITY);
    distance_[source] = 0;

    std::vector<int> parent; //references the parent node for a MST
    parent.resize(numVertices(), -1); //-1 means no parent

    //maps each vertex to a boolean visited array
    bool * visited = new bool[numVertices()];
    for( int i =0; i < numVertices(); i++) {
      visited[i]= 0; // 0 means not visited
    }

    //construct priority Que for backtracking
    //MINIMUM PRIORITY QUE but the std library is a maximum
    //The integers in the priority que represents the vertex with the shortest d[v]
    std::priority_queue< int, vector<int>, greater<int> > pque;
    pque.push(source);


   while(!(pque.empty())) {
     int node = pque.top(); //get minimum element
     //cout <<"pop from pque = " << node << endl;
     visited[node] =1;
     pque.pop();

     //get the list of all edges and iterate over them
     //current is the edge we are on; current->end is the vertex we are going to
     AdjacencyList al = incidentEdges(node);
     AdjacencyListNode * current = al.head;
     while(current != NULL ) {
       if(visited[current->end] == 0){

         //get the edge weight for each adjacent edge
         //int next = current->next->end;
         Edge * edge = getEdge(node, current->end);
         if(edge == NULL){
           current = current->next;
           continue;
         }
         if(distance_[node] + edge->weight_ < distance_[current->end] ) {

           visited[current->end] = 1;
           distance_[current->end] = distance_[node] + edge->weight_;
           parent[current->end] = node;
          // cout<< node << endl;
           pque.push(current->end);
           //cout << "push to pque =  " << current->end << endl;
         }
        }
         current = current->next;
     }
   }
   for(size_t i = 0; i < distance_.size(); i++){
     if(distance_[i] == INFINITY){
       distance_[i] = -1;
     }
   }
   return distance_;
}

//THIS IS A VECOTR THAT TELLS US WHICH NODES TO GO TOO IN ORDER
//Dijkstra's Algorithm but it returns the path
std::vector<vector<int>> Graph::dijkstra_paths(int source) {

    //output of each vertex distance from a source
    std::vector<float> distance_;
    distance_.resize(numVertices(), INFINITY);
    distance_[source] = 0;

    std::vector<int> parent; //references the parent node for a MST
    parent.resize(numVertices(), -1); //-1 means no parent
    parent[source] = source;

    //maps each vertex to a boolean visited array
    bool * visited = new bool[numVertices()];
    for( int i =0; i < numVertices(); i++) {
      visited[i]= 0; // 0 means not visited
    }

    //MINIMUM PRIORITY QUE but the std library is a maximum
    //The integers in the priority que represents the vertex with the shortest d[v]
    std::priority_queue< int, vector<int>, greater<int> > pque;
    pque.push(source);
    while(!(pque.empty())) {
      int node = pque.top(); //get minimum element
      visited[node] =1;
      pque.pop();

     //get the list of all edges and iterate over them
     //current is the edge we are on; current->end is the vertex we are going to
     AdjacencyList al = incidentEdges(node);
     AdjacencyListNode * current = al.head;
     while(current != NULL ) {
       if(visited[current->end] == 0){

         //get the edge weight for each adjacent edge
         Edge * edge = getEdge(node, current->end);
         if(edge == NULL){
           current = current->next;
           continue;
         }
         if(distance_[node] + edge->weight_ < distance_[current->end] ) {

           visited[current->end] = 1;
           distance_[current->end] = distance_[node] + edge->weight_;
           parent[current->end] = node;
           pque.push(current->end);
         }
        }
         current = current->next;
     }
   }
   // BELOW IS WHERE WE GET THE PATHS. Only thing different from the other alg

   vector< vector <int >> solution;
   //each vertex as a destination will have a shortest path
   for(int i=0; i < numVertices(); i++) {

     vector<int> path;
     path.push_back(i);
     //while the  parent is not pointing to the origin
     int j = i;
     while(parent[j] != source ){
       if(parent[j] == -1){
         path.push_back(-1);
         break;
       }
       int u = parent[j];
       path.push_back(u);

       j = u; //now go to its parent
     }
     path.push_back(source);
     solution.push_back(path); //push this path into 2d solution
   }
   return solution;
}

//brandes algo for betweenness centralitly
//derived from psuedocode at
//this algo does its own shortest path finding, but treats every edge as weight of 1
//https://www.cl.cam.ac.uk/teaching/1617/MLRD/slides/slides13.pdf?fbclid=IwAR1hXAUEqEciVF06tWkz1O9NplHqBSMiWZbwbhxwdvktqzCvqkljUqpN2wM
std::vector<float> Graph::brandes() {
  std::queue<int> queue_;
  std::stack<int> stack_;

  //distance from source
  std::vector<float> dist(getMaxVertices(), INFINITY);
  //list of predecessors on shortest path from source
  std::vector<std::vector<int>> pred(getMaxVertices(), std::vector<int>());
  //number of shortest paths from source to every vertex
  std::vector<float> sigma(getMaxVertices(), 0);
  //dependency of source on v
  std::vector<float> delta(getMaxVertices(), 0);

  std::vector<float> betweennessCentrality(getMaxVertices(), 0);

  for (int s = 0; s < getMaxVertices(); s++) {
    if (graph[s].head == NULL) continue;
    for (int w = 0; w < getMaxVertices(); w++) {
      pred[w] = std::vector<int>();
    }
    for (int t = 0; t < getMaxVertices(); t++) {
      dist[t] = INFINITY;
      sigma[t] = 0;
    }
    //shortest path algo from a source s to every vertex
    //distance to source = 0
    dist[s] = 0;
    //atleast 1 shortest path (from source to source)
    sigma[s] = 1;
    queue_.push(s);

    while (!queue_.empty()) {
      int v = queue_.front();
      stack_.push(v);
      queue_.pop();
      //for each vertex w that exists and is adjacent to v
      for (int w = 0; w < getMaxVertices(); w++) {
        if (graph[w].head == NULL) continue;
        if (!areAdjacent(v, w)) continue;

        //this part is the path discovery (w found for the first time?)
        if (dist[w] == INFINITY) {
          dist[w] = dist[v] + 1;
          queue_.push(w);
        }

        //path counting (if edge v,w is on a shortest path)
        if (dist[w] == dist[v] + 1) {
          sigma[w] = sigma[w] + sigma[v];
          pred[w].push_back(v);
        }
      }
    }
    //accumulation, something called back-propagation of dependencies
    for (int v = 0; v < getMaxVertices(); v++) {
      delta[v] = 0;
    }
    while (!stack_.empty()) {
      int w = stack_.top();
      stack_.pop();
      //for every vertex in the predecessor vector
      for (int i = 0; i < (int)pred[w].size(); i++) {
        //honestly I dont understand what this calculation is exactly
        delta[pred[w][i]] += (sigma[pred[w][i]]/sigma[w])*(1+delta[w]);
        if (w != s) {
          betweennessCentrality[w] += delta[w];
        }
      }
    }
  }
return betweennessCentrality;
}


//Dijkstra's Algorithm
std::vector<int> Graph::num_paths(int source) {

    //output of each vertex distance from a source
    std::vector<float> distance_;
    distance_.resize(numVertices(), INFINITY);
    distance_[source] = 0;

    std::vector<int> numpaths; //references the num of paths
    numpaths.resize(numVertices(), 0); //0 means no paths


    //construct priority Que for backtracking
    //MINIMUM PRIORITY QUE but the std library is a maximum
    //The integers in the priority que represents the vertex with the shortest d[v]
    std::priority_queue< int, vector<int>, greater<int> > pque;
    pque.push(source);


   while(!(pque.empty())) {
     int node = pque.top(); //get minimum element
     //cout <<"pop from pque = " << node << endl;
     pque.pop();

     //get the list of all edges and iterate over them
     //current is the edge we are on; current->end is the vertex we are going to
     AdjacencyList al = incidentEdges(node);
     AdjacencyListNode * current = al.head;
     while(current != NULL ) {

         //get the edge weight for each adjacent edge
         //int next = current->next->end;
         Edge * edge = getEdge(node, current->end);
         if(edge == NULL){
           current = current->next;
           continue;
         }

         //changed to equal so numpaths increments right
         if(distance_[node] + edge->weight_ <= distance_[current->end] ) {

           numpaths[current->end] += 1;
           distance_[current->end] = distance_[node] + edge->weight_;
           pque.push(current->end);
         }
         current = current->next;
     }
   }
   for(size_t i = 0; i < distance_.size(); i++){
     if(distance_[i] == INFINITY){
       distance_[i] = -1;
     }
   }
   return numpaths;

 }










std::vector<float> Graph::Bcentrality() {

	vector<float> Centrality;
  Centrality.resize(numVertices(), 0);


  //get centrality of each vertex
for(int i=0; i< numVertices(); i++) {

    //get all shortest paths from each node
    //currently Dijstra's path returns a reversed array

    //this is where valgrind points at a segfault

    vector< vector<int>> all_ssp = dijkstra_paths(i);

    //iterate over all destinations from i
    for(int j =0; j < numVertices(); j++ ) {
        //get the information of each path
        for(int k=1; k< (int) ( all_ssp[j].size()-1); k++ ) {

            // if the path to the jth vertice at step k equals the node
            if(i != j ){
              //im at the destination, index the betweenese
                int temp =  all_ssp[j][k];
                Centrality[temp]++;
              }
            }
        }

  }
  //from here we sould normalize the results
  for(size_t i = 0; i < Centrality.size(); i++){
    Centrality[i] = Centrality[i] / ((numVertices() - 1) * (numVertices() - 2));
  }
	return Centrality;
}
