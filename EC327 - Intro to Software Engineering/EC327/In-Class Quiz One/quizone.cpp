#include <iostream>
using namespace std;
  
/*
 * **  %E:  returns some computation based on local variables
 * */
int BigComputation(int location, int value) {
   // initialize some arbitrary variables
   int mm=9, nn=4, oo=8;
   int anArray[2]; // a local array that we will be using
               
   cout << "(Initial data in anArray[" << location << "] is " << anArray[location] << ")" << endl;
   anArray[location]=value; // change the value of the array at the given location
   cout << "(Final data in anArray[" << location << "] is " << anArray[location] << ")" << endl;
                             
   // make some computation of the local variables)
   return (10*anArray[0]+anArray[1]);
}
                                     
int main(int argc, char* argv[]) {
   // read the parameters from the command line arguments
   int firstParam=0, secondParam=0;
   if (argc>1)
   firstParam = atoi(argv[1]);  // the first command-line argument
   if (argc>2)
   secondParam = atoi(argv[2]); // the second command-line argument
   cout << "Result of the function is " << endl
   << BigComputation(firstParam,secondParam) << endl;
}
