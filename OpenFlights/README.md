# ehasson2-zayda2-colinet2-lwg2
Final Project
# OpenFlights Graph

This project implements a graph data structure that handles data from the OpenFlights dataset.  It implements a directed, Adjacency List-based graph that requires integers as vertices.  Included are basic graph functions, such as insertEdge, removeEdge, insertVertex, removeVertex, and so on.  A BFS traversal is implemented, in addition to algorithms such as Dijkstra's and Betweenness Centrality.  The user will input source and destination airports for the program to return the shortest path (by distance) between them, in addition to the shortest possible paths between the source airport and every other airport.  Due to memory and runtime issues, the program will only return betweenness centrality for the set if it is a subset of the whole 7810 airport dataset.  Test cases allow the user to verify the success of the graph implementation.

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

Future improvement includes decreasing the runtime of the betweenness centrality algorithms.
