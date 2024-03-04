//
//  SkipList.h
//  Skip_List
//
//  Copyright © Tali Moreshet. All rights reserved.
//
//
//  This is an implementation of a Skip List class, consisting of Linked Lists, which are made up of Nodes.
//  All classes are templated, such that their types may be provided at creation.
//  Nodes contain data (which doubles as key), and pointers to all directions.

#ifndef SkipList_h
#define SkipList_h

#include <iostream>
#include <cstdlib>
#include <stdlib.h>
#include <limits.h>

using namespace std;

template<class T>
class Node {
public:
    Node(const T& data);                            // constructor
    T data;                                         // node data, doubles as key
    Node<T>* next;                                  // node next pointer
    Node<T>* prev;                                  // node previous pointer
    Node<T>* up;                                    // node above pointer
    Node<T>* down;                                  // node below pointer
    void printData();                               // prints data value
    void print();                                   // prints entire node object content
};

template <class T>
class LinkedList {
public:
    LinkedList(T minVal, T maxVal);                 // constructor: accepts -infinity, +infinity values,
                                                    // creates linked list with two corresponding nodes
    ~LinkedList();                                  // destructor
    Node<T>* search(Node<T>* location, T data);     // search for data starting at location, return node <= data
    Node<T>* insert(Node<T>* location, T data);     // insert node with data at new node to be placed directly 
                                                    // after node with povided location, return new node
    void printData();                               // prints linked list data
    void print();                                   // prints linked list with all nodes pointers
    Node<T>* head;                                  // head of list
};

template <class T>
class SkipList {
public:
    SkipList(T minVal, T maxVal);                   // constructor: accepts -infinity, +infinity values,
                                                    // creates skip list with top-level only
    ~SkipList();                                    // destructor
    Node<T>* search(T data);                        // search for data, return node <= data (existing node, or location
                                                    // where new node should be inserted in the bottom-most list)
    Node<T>* insert(T data);                        // insert node with data, return pointer if inserted, NULL if error
    void printData();                               // prints skip list data
    void print();                                   // prints skip list with all nodes pointers
    LinkedList<T>* topList;                         // pointer to the top-most list
    int randSeed = 330;                             // to be used as seed for the getRand() function
    bool containsInput = false;                     // bool variable for search function to update based on if a node exists
                                                    // and insert function to distinguish between pointers to existing node or predeceesor
    int height = 0;                                 // 'height' of the skip list i.e. # of linked lists within skip list
};

//returns 0 or 1 with a 50% chance 
//When it returns 1, insert the node to next level of the skiplist
int getRand() {
    return rand() % 2;
}

/********************** From here down is the content of the LinkedList.cpp file: ***********************/

/****** Implementation of Node ******/

// Constructor
template<class T>
Node<T>::Node(const T& data)
{
    this->data = data;
    this->next = nullptr;
    this->prev = nullptr;
    this->up = nullptr;
    this->down = nullptr;
}

// Print node's data value
template <class T>
void Node<T>::printData()
{
    cout << data << " ";
}

// Print entire node object content (data value, object's address, and all pointers)
template <class T>
void Node<T>::print()
{
    cout << " * " << data << ": [addr: " << this << " next: " << this->next << " prev: " << this->prev << " up: " << this->up << " down: " << this->down << "]  ";
}


/****** Implementation of linked list ******/

/*** TO BE COMPLETED ***/

// Constructor
template<class T>
LinkedList<T>::LinkedList(T minVal, T maxVal) {
    // Dynamically allocate two pointers theHead and theTail
    // and construct nodes with the input arguments
    Node<T>* theHead = new Node<T>(minVal);
    Node<T>* theTail = new Node<T>(maxVal);

    // Set head to theHead and the tail of the list to theTail
    // Also set the prev pointer of the tail to the head
    head = theHead;
    head->next = theTail;
    head->next->prev = head;
}

// Destructor
template<class T>
LinkedList<T>::~LinkedList() {
    // Pointer curr to traverse the linked list;
    // temp to traverse linked list one 'step' ahead of curr
    // so that curr can be set to temp and be deleted
    Node<T>* curr = head;
    Node<T>* temp = head->next;
    while (temp != nullptr) {
        delete curr;
        curr = temp;
        temp = temp->next;
    }

    // Delete last node
    delete curr;
}

// Search through Linked List
template<class T>
Node<T>* LinkedList<T>::search(Node<T>* location, T data) {
    // One pointer curr to traverse the linked list,
    // and another pointer insertHere to track the location...
    // ...the data should be inserted in the case that the data is not in the list.
    Node<T>* curr = location;
    Node<T>* insertHere = head;
    while (curr != nullptr) {
        if (curr->data == data) {
            return curr;
        }
        if (data >= curr->data) {
            insertHere = curr;
        }
        curr = curr->next;
    }
    return insertHere;
}

// Insert into Linked List
template<class T>
Node<T>* LinkedList<T>::insert(Node<T>* location, T data) {
    // Dynamically allocate space for a new node for new data
    Node<T>* newNode = new Node<T>(data);

    // Insert the new node
    newNode->next = location->next;
    newNode->prev = location;
    location->next = newNode;
    newNode->next->prev = newNode;

    // Pointer to traverse linked list
    Node<T>* curr = head->next;
    while (curr != nullptr) {
        // Check if at any point the current element is smaller than the previous element;
        // This would be out of order, and thus return nullptr
        if (curr->data < curr->prev->data) {
            location->next = newNode->next;
            return nullptr;
        }
        curr = curr->next;
    }
    return newNode;
}

// Print data of each node in Linked List
template<class T>
void LinkedList<T>::printData() {
    // Pointer curr to traverse the linked list
    Node<T>* curr = head;
    while (curr != nullptr) {
        // Utilize printData() method of Node class
        curr->printData();
        curr = curr->next;
    }
}

// Print data and all addresses of each node in Linked List
template<class T>
void LinkedList<T>::print() {
    // Pointer curr to traverse the linked list
    Node<T>* curr = head;
    while (curr != nullptr) {
        // Utilize print() method of Node class
        curr->print();
        curr = curr->next;
    }
}


/****** Skip List Implementation ******/

/*** TO BE COMPLETED ***/

/*** Note: LIST and LEVEL are used interchangeably throught my documentation. ***/

// Constructor
template<class T>
SkipList<T>::SkipList(T minVal, T maxVal) {
    topList = new LinkedList<T>(minVal, maxVal); // dynamically allocate the top-most list of the skip list and set linked list pointer, topList, to point at such list
    srand(this->randSeed);
}

// Destructor
template<class T>
SkipList<T>::~SkipList() {
    Node<T>* currentList = topList->head; // current list to be deleted
    Node<T>* currentNode = currentList; // current node to be deleted
    Node<T>* tempList = currentList->down; // hold the next list to be deleted
    Node<T>* tempNode = currentNode->next; // hold the next node to be deleted

    for (int ii = 0; ii < height + 1; ii++) { // there are height+1 lists in the skip list
        while (tempNode != nullptr) {
            delete currentNode; // Note: currentList never has to be explicitly deleted because at one point currentNode points to the same memory and is deleted
            currentNode = tempNode;
            tempNode = tempNode->next;
        }
        currentNode = tempList->down;
        currentList = tempList->down;
    }
}

// Search
template<class T>
Node<T>* SkipList<T>::search(T data) {
    Node<T>* p = topList->head; // initialize a pointer 'p' to the first node in the top list (i.e. neg. infinity/sentinel)
    
    while (p != nullptr) { // condition will never be false but implementation will allow while loop to exit
        if (data == p->next->data) { // node exists with input data
            while (p->down != nullptr) {
                p = p->down; // traverse to bottom-most node
            }
            containsInput = true; // update containsInput bool variable so that insert function knows that the input data does/does not exist
            return p; // return bottom-most node containing input data
        }
        else if (data > p->next->data) { // input data is greater than next node (i.e. node containing input data should be further down the list)
            p = p->next; // go to the next node
        }
        else { // input data is less than p->next->data
            if (p->down == nullptr) { // check if p is at the bottom-most list
                return p; // based on the conditionals implemented above, p has to be at the input data's predecessor at this point
            }
            p = p->down; // go to the list one level below
        }   
    }
}

// Insert
template<class T>
Node<T>* SkipList<T>::insert(T data) {
    Node<T>* p = SkipList<T>::search(data); // initialize a pointer 'p' to the result of a search through the skip list for the input data

    if (containsInput) { // check bool variable containsInput and if true, node already exists, so return nullptr
        return nullptr;
    }
    else { // node does not exist, must insert
        // we must add a new top list of just the sentinels for the first insert operation in order to maintain the sentinel-only-top-most list property of skip lists
        if (height == 0) {
            LinkedList<T>* newList = new LinkedList<T>(INT_MIN, INT_MAX); // dynamically allocate 'newList' to point at a new linked list

            // the sentinels of newList must point to the sentinels 'below' them
            // and the sentinels 'below' must point 'up' to the newList
            newList->head->down = topList->head;
            topList->head->up = newList->head;

            Node<T>* q = topList->head; // initialize a pointer 'q' to the first node in the top list(i.e.neg.infinity / sentinel)
            while (q->next != nullptr) { // traverse the current top-most list until the 'ending' sentinel
                q = q->next;
            }
            newList->head->next->down = q;
            q->up = newList->head->next;

            topList = newList; // update topList to the newList
            height++; // increment the height now that we have one extra list
        }

        Node<T>* newNode = new Node<T>(data); // dynamically allocate pointer 'newNode' to a new node containing the input data

        newNode->next = p->next; // insert newNode to bottom-most list and correct its pointers
        p->next = newNode;
        
        int ii = 1; // initialize a variable ii to determine which list(s) above the bottom-most list also gets the newNode inserted
        while (getRand() == 1) { // while getRand == 1, insert to next list above/next level
            if (ii >= height) { // when ii is greater than height, there are no more lists 'above', so we must insert a new list
                LinkedList<T>* newList = new LinkedList<T>(INT_MIN, INT_MAX); // dynamically allocate 'newList' to point at a new linked list

                // the sentinels of newList must point to the sentinels 'below' them
                // and the sentinels 'below' must point 'up' to the newList
                newList->head->down = topList->head;
                topList->head->up = newList->head;

                Node<T>* q = topList->head; // initialize a pointer 'q' to the first node in the top list(i.e. negative infinity/sentinel)
                while (q->next != nullptr) { // traverse the current top-most list until the 'ending' sentinel
                    q = q->next;
                }
                newList->head->next->down = q;
                q->up = newList->head->next;

                topList = newList; // update topList to the newList
                height++; // increment the height now that we have one extra list
            }

            // initialize pointer 'r' and go to the beginning of list where the duplicate node should be inserted
            Node<T>* r = topList->head;
            while (r->down != nullptr) { // go the bottom-most list
                r = r->down;
            }
            for (int jj = 0; jj < ii; jj++) { // move up from the bottom-most list ii times
                r = r->up;
            }
            while (r->data < data) { // traverse list 'horizontally', and end when r is at the duplicate node's subsequent node
                r = r->next;
            }

            // create a duplicate node to be inserted into a list above the bottom-most level
            Node<T>* duplicateNode = new Node<T>(data);

            if (ii == 1) { // for the list exactly one level above the bottom-most level
                duplicateNode->down = newNode;
                newNode->up = duplicateNode;
                duplicateNode->next = r; // correct pointers after insertion
                duplicateNode->prev = r->prev;
                r->prev->next = duplicateNode;
                r->prev = duplicateNode;

            }

            if (ii > 1) { // for any duplicate nodes that need to be inserted above the bottom-most list
                // initialize pointer 's' and go to the beginning of the list one level below where the duplicate node should be inserted
                Node<T>* s = topList->head;
                while (s->down != nullptr) { // go the bottom-most list
                    s = s->down;
                }
                for (int jj = 0; jj < ii - 1; jj++) { // move up from the bottom-most list ii - 1 times to end up at the list that is exactly one level below the list for the duplicate node to be inserted
                    s = s->up;
                }
                while (s->data != data) { // traverse list horizontally until s is at the same node one level below 
                    s = s->next;
                }
                duplicateNode->down = s;
                s->up = duplicateNode;
                duplicateNode->next = r; // correct pointers after insertion
                duplicateNode->prev = r->prev;
                r->prev->next = duplicateNode;
                r->prev = duplicateNode;
            }
            ii++; // increment ii after a node has been inserted
        }

        return newNode;
    }
}

// Print skip list with data values only
template<class T>
void SkipList<T>::printData() {
    Node<T>* currentList = topList->head; // to track which list is being printed
    Node<T>* currentNode = currentList; // to print each individual node in the tracked list
    for (int ii = 0; ii < height + 1; ii++) { //
        while (currentNode != nullptr) {
            cout << currentNode->data << " ";
            currentNode = currentNode->next;
        }
        cout << endl;
        currentNode = currentList->down; // begin at new list
        currentList = currentList->down; // move the list tracker down
    }
}

// Print skip list with all information
template<class T>
void SkipList<T>::print() {
    Node<T>* currentList = topList->head; // to track which list is being printed
    Node<T>* currentNode = currentList; // to print each individual node in the tracked list
    for (int ii = 0; ii < height + 1; ii++) { //
        while (currentNode != nullptr) {
            cout << " * " << currentNode->data << ": [addr: " << currentNode << " next: " << currentNode->next << " prev: " << currentNode->prev << " up: " << currentNode->up << " down: " << currentNode->down << "]  ";
            currentNode = currentNode->next;
        }
        cout << endl;
        currentNode = currentList->down; // begin at new list
        currentList = currentList->down; // move the list tracker down
    }
}

#endif /* SkipList_h */