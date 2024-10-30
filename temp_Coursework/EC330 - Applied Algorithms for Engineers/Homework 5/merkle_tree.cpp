/*
Implementation of the merkle tree and node classes and helper function to read from an inputted file.
*/

/* 
    Acknowledgement of References Sources:
        Used in printTree method:
            https://www.geeksforgeeks.org/level-order-tree-traversal/#
            https://stackoverflow.com/questions/10258305/how-to-implement-a-breadth-first-search-to-a-certain-depth/16923440#16923440
*/

/*
    Explanation of implementation of insert function:
        My insert function is rather simple. I first use the push_back() method to add the value to-be-inserted to the
        end of my data vector. Then I call my makeLeaves() helper function, which creates the node pointers to the hashed data.
        Finally, I "reset" my root by reconstructing the merkleTree via my createLevel() recursive function.
        Please see my helper functions makeLeaves() and createLevel() for more clarity.
*/

#include "merkle_tree.h"
#include <cstdint>
#include <cmath>

using namespace std;

/* Hash function*/
inline string fnv1a(string const& text) {
    // 64 bit params
    uint64_t constexpr fnv_prime = 1099511628211ULL;
    uint64_t constexpr fnv_offset_basis = 14695981039346656037ULL;

    uint64_t hash = fnv_offset_basis;

    for (auto c : text) {
        hash ^= c;
        hash *= fnv_prime;
    }

    return to_string(hash);
}

/* Merkle Tree method for computing the hash of the concatenated hashes of children nodes
   Using the provided hash function, concatenates child hashes from left to right and returns the hash
   of the concatenation */
string merkleTree::concatenateHash(const vector<Node*>& nodes) {
    string merged;
    //concatenate hashes of all inputed nodes 
    for (const Node* node : nodes) {
        merged += node->key;
    }

    //take hash of concatenated nodes
    string mergedHash = fnv1a(merged);

    return mergedHash;
}

// Constructor of node
Node::Node(const string& val) {
    key = val;
    children = {};
    parent = nullptr;
}

// Constructor of empty Merkle tree
merkleTree::merkleTree() {
    root = nullptr;
    data = {};
}

// Constructor of Merkle tree with data
merkleTree::merkleTree(const vector<int>& data) {
    this->data = data;                                       // Save the input data vector to data
    vector<Node*> leaves = makeLeaves(data);                 // Call makeLeaves() to make the leaves

    // Call recursive function to construct Merkle tree
    root = (createLevel(leaves))[0];                         // Index 0, since createLevel returns a vector of size 1
};

// Helper function that accepts input data vector and creates corresponding leaves
vector<Node*> merkleTree::makeLeaves(const vector<int>& data) {
    maxDepth = 0;                                           // There will be at least one level since leaves are being made

    vector<Node*> leaves;

    // Traverse the given data vector
    for (auto value : data) {
        string hashedData = fnv1a(to_string(value));        // Pass the current value into the hash function
        Node* leaf = new Node(hashedData);                  // Dynamically construct a leaf with the current value hashed
        leaves.push_back(leaf);                             // Push back leaf node to the leaves vector
    }

    return leaves;
}

// Recursive helper function that accepts children nodes and creates a level of parent nodes accordingly; returns level of parent nodes
vector<Node *> merkleTree::createLevel(const vector<Node *> &children) {
    maxDepth++;                    // Add one level to maxDepth every time a level is created via the createLevel() method

    if (children.size() <= 4) {    // Base case
        string rootData = concatenateHash({children.begin(),children.end()});
        Node* theRoot = new Node(rootData);
        theRoot->children = {children.begin(),children.end()};
        for (auto child : theRoot->children){
            child->parent = theRoot;
        }
        return {theRoot};
    }

    vector<Node *> parents;                                                                     // Vector of parent nodes for the level
    for (int ii = 0; ii < (children.size()/4)*4; ii+=4) {
        string parentData = concatenateHash({children.begin()+ii,children.begin()+ii+4});       // Call concatenateHash method to get the hash of the 4 children hashes
        Node* theParent = new Node(parentData);                                                 // Dynamically construct parent node of 4-children-group
        theParent->children = {children.begin()+ii,children.begin()+ii+4};                      // Set the children member of parent to the 4-children-group
        for (auto child : theParent->children){
            child->parent = theParent;                                                          // Set the parent of the children nodes to the current parent
        }
        parents.push_back(theParent);
    }

    if (children.size() % 4 != 0) {        // For the remaining children nodes
        string parentData = concatenateHash({children.begin()+((children.size()/4)*4),children.end()});
        Node* theParent = new Node(parentData);
        theParent->children = {children.begin()+((children.size()/4)*4),children.end()};
        for (auto child : theParent->children){
            child->parent = theParent;
        }
        parents.push_back(theParent);
    }
    
    return createLevel(parents);           // Return the level of parent nodes
}

// Visualize Merkle tree (see comment @ top of file for acknowledgement of references
void merkleTree::printTree(const Node* node, int depth) {
    if (node == nullptr) {           // If it is an empty merkleTree, return without printing
        return;
    }

    if (depth > maxDepth) return;    // Cannot print a nonexistent level, so just return

    int desiredDepth = depth;        // Set the desiredDepth to the depth input

    // Find the desired depth in the case it is not specified but the node argument is NOT the root
    if (node != root) {
        queue<const Node*> nonRootQ;     // Declare a queue 'q' for Level Order Traversal/Breadth First Search
        nonRootQ.push(root);             // Enqueue the root

        int nonRootCurrDepth = 0;                // Start @ Level 0
        int nonRootElemTilDepthInc = 1;          // Level 0 is the root, so there is only one node before increasing the depth/level
        int nonRootSubseqElemTilDepthInc = 0;    // Track how many elements there are until depth increase of the SUBSEQUENT depth/level

        while (!nonRootQ.empty()) {
            const Node* nonRootCurrNode = nonRootQ.front();                      // Save the current node
            if (nonRootCurrNode == node) {                                       // When the appropriate node has been found
                desiredDepth = nonRootCurrDepth;                                 // Set the desiredDepth to the currDepth since we are at the appropriate depth/level
                break;                                                           // And break out of the while loop (and consequently the conditional
            }
            nonRootQ.pop();                                                      // Pop the current node/front element

            nonRootSubseqElemTilDepthInc += nonRootCurrNode->children.size();    // Add the amount of children to the # of elements until depth increase
            if (--nonRootElemTilDepthInc == 0) {                                 // Decrement the # of elements until depth increase,
                                                                                 // and if we are @ the last element before increasing the depth, enter conditional block

                nonRootCurrDepth++;                                              // Increase the depth
                nonRootElemTilDepthInc = nonRootSubseqElemTilDepthInc;           // Set the # of current elements until depth increase to the subsequent one
                nonRootSubseqElemTilDepthInc = 0;                                // Reset the # of elements to depth increase of the subsequent level
            }

            for (auto child : nonRootCurrNode->children) {
                nonRootQ.push(child);                                            // Enqueue all children of the current node
            }
        }
    }

    if (depth >= desiredDepth) desiredDepth = depth;                             // In the case that the depth is actually greater than
                                                                                 // the depth of the non-root node passed to the function,
                                                                                 // set the desiredDepth back to the depth input

    queue<const Node*> q;     // Declare a queue 'q' for Level Order Traversal/Breadth First Search
    q.push(root);             // Enqueue the root

    int currDepth = 0;                // Start @ Level 0
    int elemTilDepthInc = 1;          // Level 0 is the root, so there is only one node before increasing the depth/level
    int subseqElemTilDepthInc = 0;    // Track how many elements there are until depth increase of the SUBSEQUENT depth/level

    while (!q.empty()) {
        const Node* currNode = q.front();                      // Save the current node
        if (currDepth >= desiredDepth) {                       // Only print starting at desired level (depth parameter)
            for (int ii = 0; ii < currDepth; ii++) {           // Spaces for formatting
                cout << " ";
            }
            cout << "Level " << currDepth << ": ";
            cout << q.front()->key << endl;                    // Print the current node/front of the queue
        }
        q.pop();                                               // Pop the current node/front element

        subseqElemTilDepthInc += currNode->children.size();    // Add the amount of children to the # of elements until depth increase
        if (--elemTilDepthInc == 0) {                          // Decrement the # of elements until depth increase,
                                                               // and if we are @ the last element before increasing the depth, enter conditional block
            currDepth++;                                       // Increase the depth
            elemTilDepthInc = subseqElemTilDepthInc;           // Set the # of current elements until depth increase to the subsequent one
            subseqElemTilDepthInc = 0;                         // Reset the # of elements to depth increase of the subsequent level
        }

        for (auto child : currNode->children) {
            q.push(child);                                     // Enqueue all children of the current node
        }
    }
}

// Verify
void merkleTree::verify() {
    Node* oldRoot = root;                                            // Save the current root
    vector<Node*> leaves = makeLeaves(data);                         // Rebuild leaves
    root = (createLevel(leaves))[0];                                 // Rebuild tree and set root to the first (and only) element of the output vector of the recursive createLevel() method
    oldRoot->key == root->key ? cout << "True" : cout << "False";    // Print True if roots match; else print False
}

// Overwrite
void merkleTree::overwrite(int originalValue, int newValue) {
    replace(data.begin(),data.end(),originalValue,newValue);    // Use STL algorithm to swap out the originalValue with the newValue
    vector<Node*> leaves = makeLeaves(data);                    // Rebuild leaves
    root = (createLevel(leaves))[0];                            // Rebuild tree and set root to the first (and only) element of the output vector of the recursive createLevel() method
}

// Insert
void merkleTree::insert(int newValue) {
    data.push_back(newValue);                   // Push_back newValue to data vector
    vector<Node*> leaves = makeLeaves(data);    // Rebuild leaves
    root = (createLevel(leaves))[0];            // Rebuild tree and set root to the first (and only) element of the output vector of the recursive createLevel() method
}

// Print root of Merkle tree
void merkleTree::printRoot() {
    cout << root->key << endl;
}

// Return pointer to root of Merkle tree
Node* merkleTree::getRoot() {
    return root;
}

// Helper function to read from inputted file
vector<int> readIntsFromFile(const string& filename) {
    vector<int> data;
    ifstream inputFile(filename);
    int value;
    while (!inputFile.eof()) {
        inputFile >> value;
        data.push_back(value);
    }
    return data;
}