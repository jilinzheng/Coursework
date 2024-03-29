//
//  main.cpp
//
//  Created by Tali Moreshet
//

#include <iostream>
#include "Graph.h"

int main(int argc, const char * argv[]) {
  /*
  if (argc != 2)
  {
    cout << "Please supply a file name as input" << endl;
    return -1;
  }
  */
  Graph graph;
  //for part (a)
  //graph.generateGraph(argv[1]);
  graph.generateGraph("graph.txt");
  graph.print();

  //graph.addEdge(6,7,14);
//  graph.print();

  // for part (b)
  graph.numShortestPaths(0);

  return 0;
}