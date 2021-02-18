//Implement Driver Code and create a smaller test dataset
#include "Graph.h"
#include <iostream>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <iterator>
#include <string>
#include <boost/algorithm/string.hpp>

using namespace std;

Graph buildGraph(std::vector<std::string> routes, std::vector<std::string> airports) {
  int base = 10;
  int sourceIdx = 3;
  int destIdx = 5;
  Graph graph(2 * airports.size());
  int IDIdx = 0;
  int latIdx = 6;
  int longIdx = 7;
  int IATAIdx = 4;
  //array represents whether an airport has beed added to the graph yet to prevent duplicates
  vector<int> added(2 * airports.size());
  //handle airport data
  for (unsigned i = 0; i < airports.size(); i++) {
    std::string line = airports.at(i);
    istringstream iss(line);
    std::string item;
    int count = 0;
    int id;
    float latitude;
    float longitude;

    while(getline(iss, item, ',')) {
      if (count == IDIdx) {
        id = stoi(item, nullptr, base);
        if (added[id] == 0) {
          graph.insertVertex(id);
          added[id] = 1;
        }
      } else if (count == latIdx) {
        std::string::size_type sz;
        //std::cout << id << std::endl;
        latitude = stof(item, &sz);
      } else if (count == longIdx) {
        std::string::size_type sz;
        //std::cout << id << std::endl;
        longitude = stof(item, &sz);
      } else if (count == IATAIdx) {
        graph.setName(id, item);
      }
      count++;
    }

    if (added[id] == 1) {
      graph.setLatLong(id, latitude, longitude);
    }
  }

  for (unsigned i = 0; i < routes.size(); i++) {
    std::string line = routes.at(i);
    istringstream iss(line);
    std::string item;
    int count = 0;
    int source;
    int dest;
    //retrieve data from third and fifth inter-comma indices
    while(getline(iss, item, ',')) {
      if (count == sourceIdx) {
        source = stoi(item, nullptr, base);
      } else if (count == destIdx) {
        dest = stoi(item, nullptr, base);
      }
      count++;
    }
    //if source/dest do not exist in graph, add Vertex
    if (added[source] == 0) {
      added[source] = 1;
      graph.insertVertex(source);
    }
    if (added[dest] == 0) {
      added[dest] = 1;
      graph.insertVertex(dest);
    }
    //add edge source->dest
    //std::cout << source << "   " << dest << "   " << graph.distance(source, dest) << std::endl;
    graph.insertEdge(source, dest, graph.distance(source, dest));
  }



  return graph;
}

std::string getStartName() {
  std::cout << "Starting airport?"<< std::endl;
  std::string source;
  std::getline(std::cin, source);
  return source;
}

std::string getDestName() {
  std::cout << "Destination airport?"<< std::endl;
  std::string destination;
  std::getline(std::cin, destination);
  return destination;
}

int main () {
  std::ifstream routes{"routes.dat.txt"};
  std::string str;
  std::vector<std::string> routeVec;

  while (std::getline(routes, str)) {
    routeVec.push_back(str);
  }

  std::ifstream airports{"airports.dat.txt"};
  std::string str2;
  std::vector<std::string> airportVec;

  while (std::getline(airports, str2)) {
    airportVec.push_back(str2);
  }

  Graph graph = buildGraph(routeVec, airportVec);
  std::cout << "Graph built with " << graph.numVertices()  << " vertices"<< std::endl;
  // std::cout << graph.getName(0) << '\n';
  // std::cout << graph.getName(3) << '\n';

  std::string source = getStartName();
  int sid = graph.getID(source);
  // std::cout << source << '\n';
  // boost::to_upper(source);

  while (source.length() != 5 || sid == -1) {
    std::cout << "invalid airport name." << '\n';
    source = getStartName();
    // boost::to_upper(source);
    std::cout << source << '\n';
    sid = graph.getID(source);
    // std::cout << sid << '\n';
  }

  std::string destination = getDestName();
  // boost::to_upper(destination);
  int did = graph.getID(destination);

  while (destination.length() != 5 || did == -1) {
    std::cout << "invalid airport name." << '\n';
    destination = getDestName();
    // boost::to_upper(destination);
    did = graph.getID(destination);
  }
  //std::cout << "got here" << std::endl;
  //Compute Dijkstra shortest path from sid to did
  std::vector<std::vector<int>> paths = graph.dijkstra_paths(sid);
  std::vector<int> path = paths[did];
  std::cout << "Route from " << source  << " to " << destination << ":" <<  '\n';
  for (unsigned i = path.size() - 1; i > 0; i--) {
    std::cout << graph.getName(path[i]) << std::endl;
  }
  if(path.size() > 0){
    std::cout << graph.getName(did) << std::endl;
  }
  std::cout << "Shortest Paths to Other Airports via Dijkstra's: " << std::endl;
  std::vector<float> distance = graph.dijkstra(sid);
  for(unsigned i = 0; i < distance.size(); i++){
    std::cout << i << "   " << distance[i] << std::endl;
  }
  //output betweenness CENTRALITY
  std::vector<float> bc1 = graph.Bcentrality();
  std::vector<float> bc2 = graph.brandes();
  std::cout << "BC1: " << '\n';
  for (unsigned i = 0; i < bc1.size(); i++) {
    std::cout << bc1[i] <<  ", " << '\n';
  }
  std::cout << "BC2: " << '\n';
  for (unsigned i = 0; i < bc2.size(); i++) {
    std::cout << bc2[i] <<  ", " << '\n';
  }

  return 0;
}
