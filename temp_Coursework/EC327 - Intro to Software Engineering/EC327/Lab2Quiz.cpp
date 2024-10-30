#include <iostream>
#include <vector>
#include <climits>

using namespace std;

bool isPrime(int num){
	for(int ii = 2; ii < num; ii++){
		if(num % ii == 0){
			return false;
		}
	}
	return true;
}

int main(){
	vector<int> primesRem3;

	int count = 2;

	while (primesRem3.size() < 100){
		for (int ii = count; ii < INT_MAX; ii++){
			if (isPrime(ii) && (ii % 4 == 3)){
				cout << ii << endl;
				primesRem3.push_back(ii);
				count = ii+1;
				break;
			}
		}
	}
	
	return 0;
}
