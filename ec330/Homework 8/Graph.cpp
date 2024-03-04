#include "Graph.h"
#include <fstream>
#include <iostream>
#include <string>
#include <utility>
#include <limits.h>
#include <queue>
#include <algorithm>
#include <functional>

// Constructor
Graph::Graph() {
  numVertices = 0;
  numEdges = 0;
}

// Set # vertices
void Graph::initVertices(int V) {
  numVertices = V;
}

// Return # vertices
int Graph::getNumVertices() {
  return numVertices;
}

// Set # edges
void Graph::setNumEdges(int E) {
  numEdges = E;
}

// Return # edges
int Graph::getNumEdges() {
  return numEdges;
}

// Add an edge to existing vertices
void Graph::addEdge(int v, int u, int weight){
  // Save the edge & weight as a pair to both nodes' adjacency lists
  std::pair<int, int> p1, p2;
  p1 = std::make_pair(u, weight);
  p2 = std::make_pair(v, weight);
  adj[v].push_back(p1);
  adj[u].push_back(p2);
}

// Print adjacency list
void Graph::print() {
  // Traverse the adjacency list
  for (int ii = 0; ii < numVertices; ii++) {
    // Save the current list to currList
    std::vector<pair<int, int>> currList = adj[ii];

    // Print the current vertex
    std::cout << ii << ": ";

    // Traverse the list; print out edges (connections) and weights
    for (auto edge : currList) {
      std::cout << edge.first << " (" << edge.second << ") ";
    }

    std::cout << "\n";
  }
  std::cout << "\n";
}

// Construct graph from file
void Graph::generateGraph(std::string filename) {
  // Open file and verify successful open operation
  std::fstream graphData(filename);
  /*
  if (!graphData.is_open()) {
    std::cout << "File did NOT open successfully...returning..." << "\n";
    return;
  }
  std::cout << "File opened successfully!" << "\n";
  */

  // Save the # of vertices and edges from the first line of the graph data by finding the spaces
  std::string firstLine;
  getline(graphData, firstLine);
  /*
  int numVertices, numEdges;
  graphData >> numVertices >> numEdges;
  */
  int firstSpacePos = firstLine.find(' ');
  int secondSpacePos = firstLine.rfind(' ');
  int numVertices = std::stoi(firstLine.substr(0, firstSpacePos));
  int numEdges = std::stoi(firstLine.substr(firstSpacePos+1, secondSpacePos - firstSpacePos));
  Graph::initVertices(numVertices);
  Graph::setNumEdges(numEdges);

  // Allocate memory for the adjacency list using the # of vertices
  adj = new vector<pair<int, int>>[numVertices];

  while (!graphData.eof()) {
    // Get the current line and find the spaces
    std::string currLine;
    getline(graphData, currLine);
    int firstSpacePos = firstLine.find(' ');
    int secondSpacePos = firstLine.rfind(' ');

    // Use the spaces to create the nodes of interest and save the weight
    int node1 = std::stoi(currLine.substr(0, firstSpacePos));
    int node2 = std::stoi(currLine.substr(firstSpacePos+1, secondSpacePos - firstSpacePos));
    int weight = std::stoi(currLine.substr(secondSpacePos+1));

    // Save the edge & weight as a pair to both nodes' adjacency lists
    std::pair<int, int> p1, p2;
    p1 = std::make_pair(node2, weight);
    p2 = std::make_pair(node1, weight);
    adj[node1].push_back(p1);
    adj[node2].push_back(p2);
  }
}

// Print weight and NUMBER of shortest paths
// from existing source to all other vertices
void Graph::numShortestPaths(int source) {
  /*
  Implementation of Modified Dijkstra:
    My modified Dijkstra counts the paths during the 'relaxation'. I first initialize the # of paths to the source to 1,
    since there is a path from the source to itself. Then, for every relaxation, if the distance of the adjacent node v
    is GREATER than the distance of the origin node u plus the weight, I set the paths of v equal to the paths of u. If
    the distance of v is EQUAL to the distance of u plus the weight, I add the paths of u to v.
  */
  dists.resize(numVertices);
  paths.resize(numVertices);
  visited.resize(numVertices, false);

  vector<int> q;                                            // A replacement for a priority_queue so that min can be found
                                                            // (instead of complicating with Comparison in priority_queue)

  dists[source] = 0;
  paths[source] = 1;
  visited[source] = true;
  q.push_back(source);

  for (int ii = 0; ii < numVertices; ii++){
    if (ii == source) continue;
    dists[ii] = INT_MAX;                                    // + infinity initialization for all non-source nodes
    paths[ii] = 1;
    q.push_back(ii);                                        // Add node to 'q'
  }

  if (adj[source].size() == 0) {    // We started at an unconneced source
    q = {};                         // Empty the queue - no need to check for non-existent paths
  }

  while (!q.empty()) {                                      // For all the nodes
    // Extract min
    int minNode = source;                                   // Placeholder
    int minDist = INT_MAX;
    for (int ii = 0; ii < q.size(); ii++) {                 // Traverse nodes
      if (q[ii] == source) continue;                        // This is the source; don't consider
      
      if (adj[q[ii]].size() == 0) {                         // If the node has no edges (unconnected)
        minNode = q[ii];                                    // Set the minNode to the unconnected node
        break;                                              // And break out of the loop
      }

      if (!visited[q[ii]] && dists[q[ii]] < minDist) {      // If the node has NOT been visited AND the distance to that node is shorter
        minNode = q[ii];                                    // Set the minimum node to the one the minimum distance occurred on
        minDist = dists[q[ii]];                             // Set the minimum distance 
      }
    }

    // By the end of the above for loop we should have the u = minNode and its corresponding minDist
    visited[minNode] = true;                                // Set the node to visited
    
    // Search for the correct index of the just-visited node and remove it from q
    for (int ii = 0; ii < q.size(); ii++) {
      if (q[ii] == minNode) {
        q.erase(q.begin()+ii);
        break;
      }
    }

    // Relax all edges of the current minNode
    for (int ii = 0; ii < adj[minNode].size(); ii++) {
      relax(minNode, adj[minNode][ii].first, adj[minNode][ii].second);
    }
  }

  // Print the output
  std::cout << "Shortest paths from node " << source << ":\n";
  for (int ii = 0; ii < numVertices; ii++) {
    if (ii == source) continue;                             // Skip source node
    std::cout << "Distance to vertex " << ii << " is " << dists[ii] << " and there are " << paths[ii] << " shortest paths\n";
  }
}

// Relax for Dijkstra
void Graph::relax(int u, int v, int weight) {
  if (dists[v] > dists[u] + weight) {
    dists[v] = dists[u] + weight;
    paths[v] = paths[u];
    return;
  }
  else if (dists[v] == dists[u] + weight) {
    paths[v]+=paths[u];
    return;
  }

}