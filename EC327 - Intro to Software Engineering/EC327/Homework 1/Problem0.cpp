#include <iostream>
//header files with credits to cplusplus.com
#include <climits>
#include <cfloat>

using namespace std;

int main() {
	//data type: char
	char testChar;
	cout << "Minimum permissible value of char: " << CHAR_MIN << endl;
	cout << "Maximum permissible value of char: " << CHAR_MAX << endl;
	cout << "Number of bytes of memory used for storing char: " << sizeof(testChar) << endl;

	//data type: signed char
	signed char testSignedChar;
	cout << "Minimum permissible value of signed char: " << SCHAR_MIN << endl;
        cout << "Maximum permissible value of signed char: " << SCHAR_MAX << endl;
        cout << "Number of bytes of memory used for storing signed char: " << sizeof(testSignedChar) << endl;

	//data type: unsigned long
	unsigned long testUnsignedLong;
	cout << "Minimum permissible value of unsigned long: " << ULONG_MAX - ULONG_MAX << endl;
        cout << "Maximum permissible value of unsigned long: " << ULONG_MAX << endl;
        cout << "Number of bytes of memory used for storing unsigned long: " << sizeof(testUnsignedLong) << endl;

	//data type: float
	float testFloat;
	cout << "Minimum permissible value of float: " << FLT_MIN << endl;
        cout << "Maximum permissible value of float: " << FLT_MAX << endl;
        cout << "Number of bytes of memory used for storing float: " << sizeof(testFloat) << endl;

	//data type: double
	double testDouble;
	cout << "Minimum permissible value of double: " << DBL_MIN << endl;
        cout << "Maximum permissible value of double: " << DBL_MAX << endl;
        cout << "Number of bytes of memory used for storing double: " << sizeof(testDouble) << endl << endl;

	return 0;
}
