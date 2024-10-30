#include <iostream>

using namespace std;

int main(){
	float flt = 0;
	int count = 0;

	while (flt <= 400){
		flt += 0.1;
		count++;
	}

	cout << flt << endl;
	cout << flt/count << endl;

	return 0;
}
