/*
Header file for merkle tree implementation
*/

#ifndef MerkleTree_h
#define MerkleTree_h

#include <iostream>
#include <vector>
#include <string>
#include <functional>
#include <algorithm>
#include <fstream>
#include <queue>

using namespace std;

class Node {
public:
    string key;                 //store hash
    vector<Node*> children;     //store vector of children nodes
    Node(const string& val);    //constructor

    Node* parent;               // Parent of the node
};

class merkleTree {
private:
    Node* root;                                               //pointer to root node
public:
    int maxDepth = -1;                                        // Store the maximum depth of the merkleTree

    vector<int> data;                                         //store data that tree is built on
    merkleTree();                                             //constructor
    merkleTree(const vector<int>& data);                      //constructor with arguments

    vector<Node*> makeLeaves(const vector<int>& data);        // Create leaves from input data vector
    vector<Node*> createLevel(const vector<Node*>& children); // To create a new level of parents, given the children

    string concatenateHash(const vector<Node*>& nodes);       //merge and rehash nodes
    void printTree(const Node* node, int depth = 0);          //visualization of tree
    void verify();                                            //store the current root, then given current data vector,
                                                              //rebuild tree and compare the original root to the rebuilt root.
                                                              //print "True" if they match, "False" if else
    void overwrite(int originalValue, int newValue);          //override given value with newly inputted value and rehash tree
    void insert(int newValue);                                //insert new value to the end of data vector and rehash tree
    void printRoot();                                         //print root of tree

    Node* getRoot();                                          // Return pointer to the root of the tree

};

vector<int> readIntsFromFile(const string& filename);         //helper function to read from inputted file

#endif /* MerkleTree_h*/