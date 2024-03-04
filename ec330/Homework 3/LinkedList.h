//
//  LinkedList.h
//  Linked_List
//
//  Copyright © Tali Moreshet. All rights reserved.
//
//  This is an implementation of an ordered Linked List class, which is made up of Nodes.
//  All classes are templated, such that their types may be provided at creation.
//  Nodes contain data (which doubles as key), and pointers to all directions.

#ifndef LinkedList_h
#define LinkedList_h

#include <iostream>
#include <cstdlib>
#include <time.h>

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
    //  creates linked list with two corresponding nodes
    ~LinkedList();                                  // destructor
    Node<T>* search(Node<T>* location, T data);     // search for data starting at location, return node <= data
    Node<T>* insert(Node<T>* location, T data);     // insert node with data at new node to be placed directly 
    //  after node with povided location, return new node
    void printData();                               // prints linked list data
    void print();                                   // prints linked list with all nodes pointers
    Node<T>* head;                                  // head of list
};

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

#endif /* LinkedList_h */