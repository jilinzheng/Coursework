/*
	Acknowledgement of Reference Source:
		Manacher's Algorithm - Used for finding the longest palindrome in Part C
			https://www.geeksforgeeks.org/manachers-algorithm-linear-time-longest-palindromic-substring-part-4/?ref=lbp
*/

#include "Problem5.h"
#include <iostream>
#include <fstream>
#include <string>
#include <cctype>
#include <vector>
#include <chrono>

#define N 52 // Since the dictionary is case-sensitive, the alphabet will be 52 letters

using namespace std;

// return # of BU-IDs whose digits add to a number greater than 28
// BU-ID is defined as anything starting with a U and
// followed by exactly 8 digits and then a non-digit
int fiveA() {
	//auto start = chrono::high_resolution_clock::now();

	// Open the data and check that the open operation was successful
	fstream data;
	data.open("BigData.txt");//, fstream::in);
	if (!data.is_open()) return -1;

	// Procedure:
	/*
	* Look for "U"; ensure the following eight characters are DIGITS (some is_whatever function);
	* if so, sum digits and check if > 28; if so add to a running count; loop until eof
	*/
	int count = 0;                                   // A running count of how many matches there are
	char currChar;
	while (data.get(currChar)) {                     // Get the current character
		if (currChar == 'U') {                       // If we see an "U"
			char BUID[8];
			data.read(BUID, 8);                      // Read the next 9 characters
			int IDSum = 0;                           // A running sum of the ID digits
			bool isLegit = true;                     // A boolean variable that tells us whether the sequence was a valid match
			for (int ii = 0; ii < 8; ii++) {
				if (isdigit((*(BUID + ii)))) {       // Check if the current character is a digit
					IDSum += (int)(*(BUID + ii));    // If so, add the digit to the ID sum
				}
				else {                               // Not a digit!
					isLegit = false;                 // We'll mark it as not legitimate!
					break;                           // And break out of the loop
				}
			}

			if (isLegit && IDSum > 28) {             // If the 8 characters are all digits (isLegit)
				                                     // and the sum of the digits are greater than 28,
				count++;                             // Increment the running count!
			}
		}
	}
	/*
	auto stop = chrono::high_resolution_clock::now();
	auto minDuration = chrono::duration_cast<chrono::minutes>(stop - start);
	auto secDuration = (chrono::duration_cast<chrono::seconds>(stop - start)) % 60;
	cout << "Time taken by Problem 5a: " << minDuration.count() << " minutes and " << secDuration.count() << " seconds" << endl;
	*/
	return count;
}

class TrieNode {
public:
	char key;                  // The key/value stored at the TrieNode
	TrieNode* children[N];     // An array of pointers to the node's children, preallocated to N = 52
	                           // The leftmost element is 'a' and the rightmost is 'Z'
	bool isLeaf = false;       // A bool to mark whether or not a node is a leaf
	
	// Constructor
	TrieNode(char theKey) {
		key = theKey;                     // Save the input key
		
		for (auto &child : children) {    // Initialize the children
			child = nullptr;              // to nullptr
		}
	}

	// Insert
	void insert(string word) {
		TrieNode* curr = this;    // A 'current' variable to track the node (and consequently level) of the Trie
		for (int ii = 0; ii < word.size(); ii++) {
			int theIndex = word[ii] - 'a';         // Get the index of the current letter
			if (theIndex < 0) {                    // Check if the current letter is actually lowercase
				theIndex = word[ii] - 'A' + 26;    // If not, get the index of the uppercase letter
			}
			if (curr->children[theIndex] == nullptr) {                // If the child letter is not part of the Trie
				curr->children[theIndex] = new TrieNode(word[ii]);    // Create the child letter
			}
			curr = curr->children[theIndex];    // Go down a level of the Trie
		}
		curr->isLeaf = true;                    // Mark the last node of the word as a leaf node
	}

	// Search
	bool search(string target) {
		TrieNode* curr = this;

		for (int ii = 0; ii < target.size(); ii++) {
			int theIndex = target[ii] - 'a';              // Get the index of the current letter
			if (theIndex < 0) {                           // Check if the current letter is actually lowercase
				theIndex = target[ii] - 'A' + 26;         // If not, get the index of the uppercase letter
			}
			if (curr->children[theIndex] == nullptr) {    // If the child letter is not part of the Trie
				return false;
			}
			curr = curr->children[theIndex];         // Move to the next level
		}
		if (curr != nullptr && curr->isLeaf) {       // If we are not at a nullptr and it is a leaf
			return true;
		}

		return false;
	}

	// Visualize the Trie for verification
	void print() {
		if (this == nullptr) return;

		TrieNode* curr = this;
		cout << curr->key;
		for (auto child : curr->children) {
			child->print();
		}
	}
};

// Num of other valid ints
int findRemainingWords(TrieNode* dictionary, string word, int & validWords) {
	if (word.size() <= 1) return validWords;
	string smallWord = string(word.begin()+1,word.end());
	//if (smallWord.size()==0) return validWords;

	if (dictionary->search(smallWord)) {
		validWords++;
		findRemainingWords(dictionary,smallWord,validWords);
	}
}

// return # of English words from dictionary.txt that appear in the file that
// do NOT end with the same letter as my last name (Zheng -> g)
int fiveB() {
	//auto start = chrono::high_resolution_clock::now();
	
	// Open the dictionary and check that the open operation was successful
	fstream dictionary;
	dictionary.open("dictionary.txt");
	if (!dictionary.is_open()) return -1;

	TrieNode* dictionaryTrie = new TrieNode('\0');    	     // Construct first-level empty string of Trie

	// Rationale:
	/*
	* Why even bother to keep words that end with my last character?
	* Let's remove them, then whatever we have left, we'll search for in the target file.
	* And we know that any matches are legitimate.
	*/
	// Insert the entire dictionary into the Trie
	string currentWord;
	while (getline(dictionary, currentWord)) {
		if (currentWord.at(currentWord.size()-1) != 'g'){    // Only if the last letter is NOT the same last letter as my last name,
			dictionaryTrie->insert(currentWord);             // should the word be inserted
		}
	}

	// Open the data and check that the open operation was successful
	fstream data;
	data.open("TinyData.txt");
	if(!data.is_open()) return -1;

	char currChar;            // The current character
	string currWord;          // The current word
	int numValidWords = 0;    // The number of valid words
	while (data.get(currChar)) {
		if (isalpha(currChar)) {                       // If the current character is alphabetic
			currWord.push_back(currChar);              // Add the current char to the current word
			if (dictionaryTrie->search(currWord)) {    // Search the Trie for the current word;
				numValidWords++;                       // if exists, add one to the # of valid words
			}
		}
		else {
			int remainingValidWords = 0;
			int addMore = findRemainingWords(dictionaryTrie,currWord,remainingValidWords);
			numValidWords+=addMore;
			currWord = "";    // Reset the current word if we run into a non-alphabetic character
		}
	}
	/*
	auto stop = chrono::high_resolution_clock::now();
	auto minDuration = chrono::duration_cast<chrono::minutes>(stop - start);
	auto secDuration = (chrono::duration_cast<chrono::seconds>(stop - start))%60;
	cout << "Time taken by Problem 5b: " << minDuration.count() << " minutes and " << secDuration.count() << " seconds" << endl;
	*/
	return numValidWords;
}

// A helper function for Manacher's Algorithm
int min(int a, int b)
{
	int res = a;
	if (b < a)
		res = b;
	return res;
}
// Implementation of Manacher's Algorithm
string findLongestPalindrome(string word){
	int NN = word.length();    // Get the length of the input
	if (NN == 0) return "";    // If there is nothing inputted, return an empty string
	
	string returnString;       // To store the longest palindrome

	NN = 2 * NN + 1;      // Position count  
	vector<int> L(NN);    // LPS Length Array  
	L[0] = 0;
	L[1] = 1;
	int C = 1;            // Center Position  
	int R = 2;            // Center Right Position  
	int ii = 0;           // Current Right Position  
	int iMirror;          // Current Left Position  
	int maxLPSLength = 0;
	int maxLPSCenterPosition = 0;
	int start = -1;
	int end = -1;
	int diff = -1;

	for (ii = 2; ii < NN; ii++) {
		iMirror = 2 * C - ii;    // Get currentLeftPosition iMirror for currentRightPosition i  
		L[ii] = 0;
		diff = R - ii;

		if (diff > 0) L[ii] = min(L[iMirror], diff);    // If currentRightPosition i is within centerRightPosition R  

		// Attempt to expand palindrome centered at currentRightPosition i
		// For odd positions, we compare characters and if they match, increment LPS Length by ONE  
		// For even position, we just increment LPS by ONE without any character comparison  
		while (((ii + L[ii]) < NN && (ii - L[ii]) > 0) &&
			(((ii + L[ii] + 1) % 2 == 0) || (word[(ii + L[ii] + 1) / 2] == word[(ii - L[ii] - 1) / 2]))) {
			L[ii]++;
		}

		if (L[ii] > maxLPSLength) {    // Track maxLPSLength  
			maxLPSLength = L[ii];
			maxLPSCenterPosition = ii;
		}
 
		if (ii + L[ii] > R) {    // If the palindrome is centered at currentRightPosition i,  
			C = ii;              // adjust the centerPosition C based on the expanded palindrome,
			R = ii + L[ii];      // and expand beyond centerRightPosition R
		}
	}

	start = (maxLPSCenterPosition - maxLPSLength) / 2;
	end = start + maxLPSLength - 1;

	for (ii = start; ii <= end; ii++) {
		returnString.push_back(word[ii]);
	}

	return returnString;
}

// return length of longest palindrome within text
// ALL characters count including spaces
int fiveC() {
	//auto start = chrono::high_resolution_clock::now();

	// Open the data and check that the open operation was successful
	fstream data;
	data.open("BigData.txt");
	if (!data.is_open()) return -1;
	
	string allData;
	getline(data, allData);    // Store the entire data in one string

	int length = 0;            // Track the length of the longest palindrome
	string longestPalindrome = findLongestPalindrome(allData);

	//auto stop = chrono::high_resolution_clock::now();
	//auto minDuration = chrono::duration_cast<chrono::minutes>(stop - start);
	//auto secDuration = (chrono::duration_cast<chrono::seconds>(stop - start)) % 60;
	//cout << "Time taken by Problem 5c: " << minDuration.count() << " minutes and " << secDuration.count() << " seconds" << endl;

	//cout << "The longest palindrome is: " << longestPalindrome << endl;
	length = longestPalindrome.length();

	return length;
}
