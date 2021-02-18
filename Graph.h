
#ifndef _GRAPH_
#define _GRAPH_

#include <vector>
#include <list>
#include <utility>
#include <string>
//may need more
using namespace std;

class Graph
{
    class AdjacencyListNode {
      public:
        AdjacencyListNode();
        int end;
        AdjacencyListNode * next;
    };

    class AdjacencyList {
      public:
        AdjacencyList();
        AdjacencyListNode * head;
    };

    class Edge {
      public:
        Edge();
        Edge(int uno, int dos, float weight);
        int source;
        int dest;
        float weight_;
        string label;
        bool cross;
        bool discovery;
    };

    public:
        Graph();
        Graph(int num);
        void BFS(int n);
        void insertEdge(int source, int dest, float weight);
        void removeEdge(int source, int dest);
        void insertVertex(int n);
        void removeVertex(int n);
        AdjacencyList incidentEdges(int n);
        bool areAdjacent(int source, int dest);
        //helper functions below
        AdjacencyListNode * newAdjacencyListNode(int n);
        int numVertices();
        int getMaxVertices();
        Edge * getEdge(int source, int dest);

        //Algorithms and necessary Functions
        std::vector<float> dijkstra(int source);
        vector<vector<int>> dijkstra_paths(int source);
        vector<int> num_paths(int source);
        void setLatLong(int n, float latitude, float longitude);
        float distance(int source, int dest);
        std::vector<float> brandes();
        std::vector<float> Bcentrality();
        void setName(int n, std::string name);
        int getID(std::string name);
        std::string getName(int id);


      private:
        //array of adjacency lists
        AdjacencyList* graph;
        //number of vertices
        int vertices;
        //max number of vertices
        int maxVertices;
        //vector of edges
        vector<Edge*> edges;

        vector<float> latitudes;
        vector<float> longitudes;
        vector<string> names;

};
#endif
