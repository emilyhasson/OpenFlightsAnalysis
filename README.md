# OpenFlights Analysis
CS225 Final Project
# OpenFlights Graph

This project implements a graph data structure that handles data from the OpenFlights dataset. It implements a directed, Adjacency List-based graph that requires integers as vertices. Included are basic graph functions, such as insertEdge, removeEdge, insertVertex, removeVertex, etc., as well as a BFS traversal. The program uses Dijkstra's algorithm to compute shortest path and a version of the Betweenness Centrality algorithm to indicate the centrality or importance of a given airport. The user is able to input source and destination airport codes for which the program will return the shortest path (by distance) between them, in addition to the shortest possible paths between the source airport and every other airport and a resulting measure of centrality. Comprehensive test cases allow the user to verify the success of the graph implementation.

## Installation

No installation required; simply navigate to the proper directory.

## Data

Each line of the airport data must be written as such:
```c
ID,\N,\N,\N,"3 letter IATA code",\N,latitude,longitude
```
Latitude and longitude should be of type float, ID should be of type int, and the IATA code should be 5 characters, including the quotation marks.  Any extraneous information from OpenFlights or the included txt files does not matter.

Each line of the route data must be written as such:
```c
\N,\N,"source 3 letter IATA code",source ID,"destination 3 letter IATA code",dest ID
```

The IATA codes should be 5 characters each, including the quotation marks.  The IDs should be of type int.  Both data can be found from OpenFlights or the included txt files.  Any extraneous information does not matter.

In the file main.cpp, lines 110 and 118 can be altered to utilize other datasets of type txt.

## Usage

Testing
```bash
make test && ./test #will compile the test file and run the executable test
```
Running the Program

The following will compile executable and prompt user to enter 3 letter IATA code for both source and destination airports, preceded and followed by " (Example: "CMI" would be the 5 character input).  If the dataset is too large for betweenness centrality to run, use a smaller subset (provided in file airports.dat.txt, as opposed to moreAirports.txt).  
```bash
make && ./main 
```

## Roadmap

Future improvement includes decreasing runtime of the betweenness centrality algorithms.
