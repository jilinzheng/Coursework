#include <iostream>
#include <string>

using namespace std;

int main(){
	string userInput;
	cout << "GIMME AN INTEGER: ";
	cin >> userInput;

	int inputSize = userInput.length();
	
	bool result = true;

	for (int i = 0; i < inputSize-1; i++){
		if(!(userInput[i+1] > userInput[i])){
			result = false;
			break;
		}
	}

	if (result){
		cout << "in order" << endl;
	} else {
		cout << "not in order" << endl;
	}
	
	return 0;
}
