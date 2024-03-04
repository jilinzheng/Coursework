#include "Huffman.h"
#include <string>
#include <queue>
#include <unordered_map>
#include <vector>
#include <iostream>
#include <utility>
#include <fstream>

Node::Node(std::string theKey, int theFreq) {
  key = theKey;
  freq = theFreq;
  parent = nullptr;
  leftChild = nullptr;
  rightChild = nullptr;
}

Tree::Tree(Node* theRoot) {
  root = theRoot;
}

Huffman::Huffman() {
  finalHuffmanTree = nullptr;
  finalHuffmanTable = {};
}

// Comparison functor for trees, used in the priority_queue
class compareTrees {
public:
  bool operator() (Tree* l, Tree* r) const {
    return l->root->freq > r->root->freq;
  }
};

void Huffman::buildHuffmanTree(char* characters, int* freq, int size) {
  maxSize = size;
  std::vector<Tree*> trees;

  for (int ii = 0; ii < size; ii++) {
    //std::cout << characters[ii] << "\n";
    Node* currCharNode = new Node({characters[ii]}, freq[ii]);    // Create a character node 
    Tree* currTree = new Tree(currCharNode);                      // Create a single-node tree from the node
    trees.push_back(currTree);                                    // Add the tree to a vector of trees
  }

  // At this point the single-node binary trees for each char have been created.

  // Create the priority queue / min heap for the Huffman Code algorithm.
  std::priority_queue<Tree*,std::vector<Tree*>, compareTrees> minTrees(trees.begin(),trees.end());

  // A sanity check. 
  /*
  while (!minTrees.empty()) {
    std::cout << minTrees.top()->root->key << " : " << minTrees.top()->root->freq << "\n";
    minTrees.pop();
  }
  */
  // We are good.

  while (minTrees.size() > 1) {
    // Save the top two trees (the lowest frequency trees)
    Tree* leftTree = minTrees.top();
    //std::cout << "The left tree is: " << leftTree->root->key << "\n";
    minTrees.pop();
    Tree* rightTree = minTrees.top();
    //std::cout << "The right tree is: " << rightTree->root->key << "\n";
    minTrees.pop();

    // To 'fix' order due to strict weak ordering of priority_queue
    if (leftTree->root->key.size() == rightTree->root->key.size() &&
        leftTree->root->freq == rightTree->root->freq) {
      std::swap(leftTree,rightTree);
    }

    // Create the new tree with subtrees leftTree and rightTree
    std::string newKey = leftTree->root->key + rightTree->root->key;
    int newFreq = leftTree->root->freq + rightTree->root->freq;
    Node* newRoot = new Node(newKey, newFreq);
    Tree* newTree = new Tree(newRoot);

    // Set the relationships
    newRoot->leftChild = leftTree->root;
    newRoot->rightChild = rightTree->root;
    leftTree->root->parent = newRoot;
    rightTree->root->parent = newRoot;

    // Check the new node
    //std::cout << "The new node's key: " << newKey << "\n";

    minTrees.push(newTree);
  }

  // At this point, we should only have one tree left
  //std::cout << minTrees.top()->root->key << " : " << minTrees.top()->root->freq << "\n";
  finalHuffmanTree = minTrees.top();  

  // Populate the finalHuffmanTable
  std::string currHuffmanCode = "";
  populateHuffmanTable(finalHuffmanTree->root, currHuffmanCode);
  /*
  for (auto row : finalHuffmanTable) {
    std::cout << row.first << " : " << row.second << "\n";
  }
  */
}

// Helper function (and variable) to populate the finalHuffmanTable (char [:] Huffman Code)
void Huffman::populateHuffmanTable(Node* theRoot, std::string currHuffmanCode) {
  Node* root = theRoot;
  //std::string currHuffmanCode = theCurrHuffmanCode;

  if (root == nullptr) {
    currHuffmanCode.pop_back();
    return;
  }

  //currHuffmanCode.push_back('0');
  populateHuffmanTable(root->leftChild, currHuffmanCode + "0");
  //finalHuffmanTable[root->key[0]] = currHuffmanCode;
  //finalHuffmanTable[root->key[0]] = populateHuffmanTable(root->leftChild, currHuffmanCode + "0");

  //currHuffmanCode.push_back('1');
  populateHuffmanTable(root->rightChild, currHuffmanCode + "1");
  //if (finalHuffmanTable.find(root->key[0]) == finalHuffmanTable.end()) {
    //finalHuffmanTable[root->key[0]] = currHuffmanCode;
  //}
  if (root->leftChild == nullptr && root->rightChild == nullptr) {
    finalHuffmanTable[root->key[0]] = currHuffmanCode;
  }


  //finalHuffmanTable[root->key[0]] = populateHuffmanTable(root->rightChild, currHuffmanCode + "1");
}

// Comparison functor for strings as a part of a pair in hashmaps, used in priority_queue
class compareCodeLengths {
public:
  bool operator() (std::pair<char, std::string> l, std::pair<char, std::string> r) const {
    return l.second.size() > r.second.size();
  }
};

void Huffman::printCodes() {
  // Use a priority_queue to sort the hashmap for printing
  std::priority_queue<std::pair<char, std::string>, std::vector<std::pair<char,std::string>>, 
       compareCodeLengths> sortedHuffmanTable(finalHuffmanTable.begin(), finalHuffmanTable.end());

  while (sortedHuffmanTable.size() > 1) {
    std::cout << sortedHuffmanTable.top().first << " " << sortedHuffmanTable.top().second << "\n";
    sortedHuffmanTable.pop();

    // The last two characters may be 'out of order' because of strict weak ordering
    if (sortedHuffmanTable.size() < 3) {    
      std::pair<char, std::string> firstPair = sortedHuffmanTable.top();
      sortedHuffmanTable.pop();
      std::pair<char, std::string> secondPair = sortedHuffmanTable.top();
      sortedHuffmanTable.pop();

      if (firstPair.second.length() == secondPair.second.length()) {
        //std::swap(firstPair, secondPair);
      }

      // The first of the pair is the character and the second is the corresponding Huffman code
      std::cout << firstPair.first << " " << firstPair.second << "\n";
      std::cout << secondPair.first << " " << secondPair.second << "\n";
    }
  }
}

void Huffman::decodeText(const char* filename) {
  std::cout << "\n";
  std::fstream encodedFile;
  encodedFile.open(filename);

  // Check that the file opened successfully
  if (!encodedFile.is_open()) {
    std::cout << "Error opening the file...returning.";
    return;
  }

  // Create a 'flipped' finalHuffmanTable to quickly search encoded sequences
  std::unordered_map<std::string,char> flippedHuffmanTable = {};
  for (auto entry : finalHuffmanTable) {
    flippedHuffmanTable[entry.second] = entry.first;
  }

  char bit;
  std::string currentEncodedSeq = "";
  while (encodedFile.get(bit)) {
    currentEncodedSeq.push_back(bit);

    if (flippedHuffmanTable.find(currentEncodedSeq) != flippedHuffmanTable.end()) {
      std::cout << flippedHuffmanTable.at(currentEncodedSeq);
      currentEncodedSeq = "";
    }

    /*
    if (currentEncodedSeq.back() == '0' || currentEncodedSeq.size() == maxSize) {    // We have reached a character's complete sequence
      std::cout << flippedHuffmanTable.at(currentEncodedSeq);                  // Print the character by searching the hashmap
      currentEncodedSeq = "";                                                  // Reset string tracking the encoded sequence
    }
    */
  }

  std::cout << "\n";
}