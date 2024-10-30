#include <iostream>
#include <vector>
#include <string>

using namespace std;

template  <typename Foo, int sz=5>
        int bar(Foo x) {
            vector<Foo> y;
            y.push_back(x);
            cout << y.size()+sz << endl;
        }

int main(){
	bar<string,0>("hi");

	return 0;
}
