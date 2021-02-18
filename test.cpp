// #include "../Catch/catch.hpp"
#include "Catch/catch.hpp"
#include "Graph.h"
#include <iostream>
TEST_CASE("Graph::insertVertex", "[weight = 1]") {

    Graph graph(13000);
    graph.insertVertex(5);
    graph.insertVertex(7);

    REQUIRE(2 == graph.numVertices());
}

TEST_CASE("Graph::insertEdge", "[weight = 1]") {

    Graph graph(13000);
    graph.insertVertex(5);
    graph.insertVertex(7);
    graph.insertVertex(9);

    graph.insertEdge(5, 9, 1);
    graph.insertEdge(9, 5, 1);


    REQUIRE(graph.getEdge(5, 9) != NULL);
    REQUIRE(graph.getEdge(9, 5) != NULL);
    REQUIRE(graph.getEdge(5, 7) == NULL);

    REQUIRE(graph.areAdjacent(5, 9));
    REQUIRE(graph.areAdjacent(9, 5));
}

TEST_CASE("Graph::removeVertex simple", "[weight = 1]") {

    Graph graph(13000);
    graph.insertVertex(5);
    graph.insertVertex(7);
    graph.insertVertex(9);

    graph.insertEdge(5, 7, 1);
    graph.insertEdge(5, 9, 1);

    graph.removeVertex(5);

    REQUIRE(!graph.areAdjacent(5, 9));
    REQUIRE(!graph.areAdjacent(5, 7));

}

TEST_CASE("Graph::removeEdge simple", "[weight = 1]") {

    Graph graph(13000);
    graph.insertVertex(5);
    graph.insertVertex(7);
    graph.insertVertex(9);

    graph.insertEdge(5, 9, 1);
    graph.insertEdge(5, 7, 1);

    graph.removeEdge(5, 9);

    REQUIRE(!graph.areAdjacent(5, 9));
    REQUIRE(graph.areAdjacent(5, 7));

}

TEST_CASE("Graph::BFS visits all edges", "[weight = 1]") {

    Graph graph(13000);
    for (int i = 1; i <= 6; i++) {
      graph.insertVertex(i);
    }

    graph.insertEdge(1, 2, 1);
    graph.insertEdge(1, 5, 1);
    graph.insertEdge(1, 6, 1);
    graph.insertEdge(2, 3, 1);
    graph.insertEdge(2, 5, 1);
    graph.insertEdge(3, 4, 1);

    graph.BFS(1);

    std::string path;
    for (int i = 1; i <= 6; i++) {
      for (int j = 1; j <= 6; j++) {
        if (graph.getEdge(i, j) != NULL && (graph.getEdge(i, j)->discovery || graph.getEdge(i, j)->cross)){
          path = path + graph.getEdge(i, j)->label + " ";
        }
      }
    }
    REQUIRE(path == "12 15 16 23 25 34 ");
}

TEST_CASE("Graph::dijkstra behaves as expected", "[weight = 1]"){
  Graph graph(9);
  for(int i = 0; i < 9; i++){
    graph.insertVertex(i);
  }
  graph.insertEdge(0, 1, 4);
  graph.insertEdge(0, 7, 8);
  graph.insertEdge(1, 2, 8);
  graph.insertEdge(1, 7, 11);
  graph.insertEdge(2, 3, 7);
  graph.insertEdge(2, 8, 2);
  graph.insertEdge(2, 5, 4);
  graph.insertEdge(3, 4, 9);
  graph.insertEdge(3, 5, 14);
  graph.insertEdge(4, 5, 10);
  graph.insertEdge(5, 6, 2);
  graph.insertEdge(6, 7, 1);
  graph.insertEdge(6, 8, 6);
  graph.insertEdge(7, 8, 7);
  std::vector<float> distances = graph.dijkstra(4);
  std::string dist;
  for(int i = 0; i < 9; i++){
    dist = dist + std::to_string((int)distances[i]) + " ";
  }
  REQUIRE(dist == "-1 -1 -1 -1 0 10 12 13 18 ");
}

TEST_CASE("Graph::Brandes", "[weight = 1]") {

    Graph graph(10);
    for (int i = 1; i <= 6; i++) {
      graph.insertVertex(i);
    }

    graph.insertEdge(1, 2, 1);
    graph.insertEdge(1, 5, 1);
    graph.insertEdge(1, 6, 1);
    graph.insertEdge(2, 3, 1);
    graph.insertEdge(2, 5, 1);
    graph.insertEdge(3, 4, 1);

    //the output is an array the size of the max possible vertices in the graph
    //so if vertex 2 has 3 shortest paths through it, the result[2] == 3
    std::cout << endl << "BC ARRAY" << endl << "__________" << endl;
    std::vector<float> result = graph.brandes();
    for (int i = 0; i < (int)result.size(); i++) {
      std::cout << "vertex " << i << ":" << result[i] << endl;
    }

    // REQUIRE(result == {1, 2, 3, 4, 5});
}

TEST_CASE("Graph::Brandes2", "[weight = 1]") {

    Graph graph(10);
    for (int i = 1; i <= 10; i++) {
      graph.insertVertex(i);
    }

    for (int i = 2; i <= 10; i++) {
      graph.insertEdge(1, i, 1);
    }

    //the output is an array the size of the max possible vertices in the graph
    //vertices with higher degrees are more "central"
    std::cout << endl << "BC ARRAY" << endl << "__________" << endl;
    std::vector<float> result = graph.brandes();
    for (int i = 0; i < (int)result.size(); i++) {
      std::cout << "vertex " << i << ":" << result[i] << endl;
    }

    for (int i = 0; i < 10; i++) {
      if (i != 1) REQUIRE(result[1] > result[i]);
    }
}
