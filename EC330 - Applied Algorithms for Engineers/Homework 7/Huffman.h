#ifndef Huffman_h
#define Huffman_h

#include <string>
#include <queue>
#include <unordered_map>

class Node {
public:
  std::string key;    // The char or sequence of chars
  int freq;           // The frequency of such char/sequence
  Node* parent;
  Node* leftChild;
  Node* rightChild;

  Node(std::string theKey, int theFreq);    // Constructor
};

class Tree {
public:
  Node* root;             

  Tree(Node* theRoot);    // Constructor
};

class Huffman {
private:
  int maxSize;
  Tree* finalHuffmanTree;
  std::unordered_map<char,std::string> finalHuffmanTable;
public:
  Huffman();    // Constructor

  /* 
  accepts an array of characters, an array of integer
  frequencies, and the integer size of these arrays, and constructs a Huffman Tree
  from the input
  */
  void buildHuffmanTree(char* characters, int* freq, int size);

  void populateHuffmanTable(Node* theRoot, std::string theCurrHuffmanCode);

  /*
  prints the Huffman tree in a table format
  */
  void printCodes();

  /*
  accepts a pointer to a file name containing a Huffman code encoded
  text, and prints out the plain text version of the file.
  */
  void decodeText(const char* filename);
};

#endif 