#include <iostream>

using namespace std;

int main () {
	float flt1 = 0, test1 = 0.125, flt2 = 0, test2 = 0.15, flt3 = 0, test3 = 0.984375;
	int count = 0;

	while (flt1 <= 500000){
		flt1 += test1;
		count++;
	}

	cout << flt1/count << endl;

	count = 0;
	while (flt2 <= 500000){
		flt2 += test2;
		count++;
	}

	cout << flt2/count << endl;

	count = 0;
	while (flt3 <= 500000){
		flt3 += test3;
		count++;
	}

	cout << flt3/count << endl;

	return 0;
}
