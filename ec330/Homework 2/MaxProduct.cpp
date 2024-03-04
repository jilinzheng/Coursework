/*
    Jilin Zheng
    U49258796
*/

#include "MaxProduct.h"

MaxProductClass::MaxProductClass() {}

long MaxProductClass::MaxProduct(std::string file) {
    // Create a fstream variable input and have it open the input 'file'
    std::ifstream input(file);

    // Check if 'file' is actually open; if not, return -1
    if (!input.is_open()) {
        std::cout << "FILE DID NOT OPEN SUCCESSFULLY!" << std::endl;
        return -1;
    }

    // Create 2-D vector and store inputs onto 2-D vector,
    // skipping to the next line (and vector) when -999999 is reached
    std::vector<std::vector<long>> elements;
    std::vector<long> elementLine;
    long element;
    while (input >> element) {
        if (element == -999999) {
            elements.push_back(elementLine);
            elementLine.erase(elementLine.begin(), elementLine.end());
            continue;
        }
        elementLine.push_back(element);
    }

    // Create max variable that will be reassigned on each new line;
    // loop through each line of 2-D vector, setting max to the first term initially,
    // then updating it if a term or product of up to three terms is greater than max
    std::vector<long> maxProducts;
    for (int ii = 0; ii < elements.size(); ii++) {
        long max = elements[ii][0];
        // Check single terms
        for (int jj = 0; jj < elements[ii].size(); jj++) {
            if (elements[ii][jj] > max)
                max = elements[ii][jj];
        }
        // Check product of two terms; continue if line contains less than two terms
        if (elements[ii].size() < 2) {
            maxProducts.push_back(max);
            continue;
        }
        for (int jj = 0; jj < elements[ii].size() - 1; jj++) {
            if (elements[ii][jj] * elements[ii][jj + 1] > max)
                max = elements[ii][jj] * elements[ii][jj + 1];
        }
        // Check product of three terms; continue if line contains less than three terms
        if (elements[ii].size() < 3) {
            maxProducts.push_back(max);
            continue;
        }
        for (int jj = 0; jj < elements[ii].size() - 2; jj++) {
            if (elements[ii][jj] * elements[ii][jj + 1] * elements[ii][jj + 2] > max)
                max = elements[ii][jj] * elements[ii][jj + 1] * elements[ii][jj + 2];
        }
        maxProducts.push_back(max);
    }

    // Use STL algorithm to return the largest of vector maxProducts
    for (long product : maxProducts) {
        std::cout << product << std::endl;
    }
    return *(std::max_element(maxProducts.begin(), maxProducts.end()));
};