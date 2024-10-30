/*
    Jilin Zheng
    jilin@bu.edu
*/

#include <iostream>
#include <stack>
#include <string>

using namespace std;

bool legalBrackets(string line){
    stack<char> lineStack;
    for (auto character : line){
        if (line.length() == 0) return true;
        
        /* Push character onto stack if it is a bracket i.e. (,{,[. */
        if (character == '(' || character == '{' || character == '['){
            lineStack.push(character);
        }

        /* Once a bracket character that is supposed to have a matching first part is seen,
        check the character vs. the top of the stack. If there is nothing at the top of the stack,
        something is wrong i.e. illegal. */
        if (character == ')'){
            if (lineStack.size() == 0 || lineStack.top() != '(') return false;
            else lineStack.pop();
        }
        if (character == '}'){
            if (lineStack.size() == 0 || lineStack.top() != '{') return false;
            else lineStack.pop();
        }
        if (character == ']'){
            if (lineStack.size() == 0 || lineStack.top() != '[') return false;
            else lineStack.pop();
        }
    }

    /* If the stack is not empty, that means there was a bracket that did not have a 'partner,'
    which is illegal. */
    if (lineStack.size() != 0) return false;

    return true;
}

int main(int argc, char* argv[]) {
    /* Case for no additional input arguments or empty string (no characters betwen quotations). */
    if (argc == 1) cout << "legal" << endl;

    /* Loop through all arguments and print whether legal or illegal based on return value of legalBrackets(). */
    for (int ii = 1; ii < argc; ii++) {
        if (legalBrackets(argv[ii])) cout << "legal" << endl;
        else cout << "illegal" << endl;
    }

    return 0;
}