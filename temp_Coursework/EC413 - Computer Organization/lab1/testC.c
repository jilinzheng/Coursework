// SOLUTION FILE


#include <stdio.h>
#define MATRIX_DIM 10


void printMatrix(int matrix[10][10]) {
  printf("\n");
  int i = 0, j = 0;
  for (; i < MATRIX_DIM; ++i) {
    for (; j < MATRIX_DIM; ++j) {
      printf("%d ", matrix[i][j]);
    }
    printf("\n");
    j = 0;
  }
}


int main() {
  int var_int;                    // 2
  char var_char;
  float var_float;
  double var_double; 
  long int var_long_int;
  short int var_short_int;
  unsigned short int var_unsigneded_short_int;

  unsigned char uchar1, uchar2;   // 3
  signed char schar1, schar2;

  int x, y, l, r;                       // 4

  char i;                         // 5
  char shift_char;

  int a[10] = {0,10,20,30,40,50,60,70,80,90};    // 6

  int b[10], c[10];               // 7
  int *ip, *ip2;
  int j, k;

  char AString[] = "HAL";           // 8


  // 1 -- change "World" to your name
  printf("\n\nPART 1 ---------\n");

  printf("\nHello Jilin!");


  // 2 -- find sizes of the other C datatypes
  printf("\n\nPART 2 ----------\n");

  printf("\nsize of data type int = %d ", sizeof(var_int));
  printf("\nsize of data type char = %d ", sizeof(var_char));
  printf("\nsize of data type float = %d ", sizeof(var_float));
  printf("\nsize of data type double = %d ", sizeof(var_double));
  printf("\nsize of data type long int = %d ", sizeof(var_long_int));
  printf("\nsize of data type short int = %d ", sizeof(var_short_int));
  printf("\nsize of data type unsigned short int = %d ", sizeof(var_unsigneded_short_int));


  // 3 -- explore signed versus unsigned datatypes and their interactions
  printf("\n\nPART 3 ----------\n");

  uchar1 = 0xFF; // = 255
  uchar2 = 0xFE; // = 254
  schar1 = 0xFF;
  schar2 = 0xFE;
  printf("\nuchar1 = %d ", uchar1);
  printf("\nschar1 = %d ", schar1);
  printf("\n\nuchar1 and schar1 have different outputs despite being assigned the same value because the former is unsigned and the latter is signed. 0xFF is 255 in decimal, which makes sense for uchar1 with a range from 0 to 255. schar1 ends up being -1 because it only has a range from -128 to 127, so any number above 127 overflows and wraps around to -128 and continues upwards.");

  printf("\n\nComparing uchar1 and uchar2:");
  if (uchar1 > uchar2) {
    printf("\nlarger value is uchar1 = %d", uchar1);
  } else {
    printf("\nlarger value is uchar2 = %d,", uchar2);
  }

  printf("\n\nComparing schar1 and schar2:");
  if (schar1 > schar2) {
    printf("\nlarger value is schar1 = %d", schar1);
  } else {
    printf("\nlarger value is schar2 = %d,", schar2);
  }

  printf("\n\nFor reference: uchar1 = %d, uchar2 = %d, schar1 = %d, schar2 = %d", uchar1, uchar2, schar1, schar2);
  printf("\n\nComparing schar1 and uchar1:");
  if (schar1 > uchar1) {
    printf("\nlarger value is schar1 = %d", schar1);
  } else {
    printf("\nlarger value is uchar1 = %d", uchar1);
  }
  printf("\n\nComparing schar1 and uchar2:");
  if (schar1 > uchar2) {
    printf("\nlarger value is schar1 = %d", schar1);
  } else {
    printf("\nlarger value is uchar2 = %d", uchar2);
  }
  printf("\n\nComparing schar2 and uchar1:");
  if (schar2 > uchar1) {
    printf("\nlarger value is schar2 = %d", schar2);
  } else {
    printf("\nlarger value is uchar1 = %d", uchar1);
  }
  printf("\n\nComparing schar2 and uchar2:");
  if (schar2 > uchar2) {
    printf("\nlarger value is schar2 = %d", schar2);
  } else {
    printf("\nlarger value is uchar2 = %d", uchar2);
  }
  printf("\n\nWhen mixed signed and unsigned types are compared, the value of the types are compared, instead of the actual bits that make up the variables.");

  printf("\n\nschar1 + schar2 = %d", schar1 + schar2); // expecting -1 + -2 = -3
  printf("\n\nThis is what I expected. Signed chars range from -128 to 127. Since schar1 is set to 0xFF (255), it is fine from 0 to 127, but wraps around after 127 to -128, and continues from there, ultimately resulting as -1. Similarly, schar2 will wrap around to -2. Thus the result -1 + (-2) = -3.");

  printf("\n\nuchar1 + uchar2 = %d", uchar1 + uchar2); // expecting 255 + 254 = 254 (range capped at 255)
  printf("\n\nThis was NOT what I expected. I expected 254 since unsigned chars range from 0 to 255 and so it would cap, but digging around a little, it seems that there exists something known as integer promotions prior to the arithmetic operation going through, which means my unsigned chars become ints which are 4 bytes, with a huge range from -2147483648 to 2147483647. This would certainly allow the result to be a measly 509.");
  printf("\n\nschar1 + uchar1 = %d", schar1 + uchar1); // expecting -1 + 255 = 254
  printf("\n\nI expected this one. Despite my note of integer promotions previously, the values are still retained, so the operation carries out fine and predictably.");


  // 4 -- Booleans
  printf("\n\nPART 4 ----------\n");

  x = 1; y = 2;

  l = 0; r = 1;
  printf("\nBoolean TRUE = %d", l || r);
  printf("\nBoolean FALSE = %d", l && r);

  printf("\n\nThe size of a 'Boolean' is %d bytes.", sizeof(l || r));

  printf("\n\n1 & 2 = %d", x & y);
  printf("\n1 && 2 = %d", x && y);
  
  printf("\n\n~1 = %d", ~x);
  printf("\n!1 = %d", !x);


  // 5 -- shifts
  printf("\n\nPART 5 ----------\n");

  shift_char = 15;
  i = 1;

  for (; i < 33; ++i) {
    printf("\nshift_char (= 15) << %d = %d", i, shift_char << i);
    printf("\nshift_char (= 15) >> %d = %d", i, shift_char >> i);
  }

  printf("\n\nWhen I shift shift_char left more than 3 places and/or more than 7 places, I end up doubling shift_char repeatedly. I would not have expected this, had I not known about integer promotions, which I found out above in part 3. If I did not know that shift_char would be promoted to an 4-byte int before being operated on, I would assume it remains a 1-byte char, resulting in shifts to the left greater than 3, to wrap back around, i.e., the range of shift_char would be from -128 to 127, and shifts to the left greater than 7, to repeat the same pattern of 'shifting results' I had seen previously, as 1 byte = 8 bits and I would have essentially shifted back to the start, where shift_char = 15.");
  
  // HINT: If you cannot observe any "interesting" results using the above statement,
  // try assigning the shifted value to a different variable and then printing the new 
  // variable.


  // 6 -- pointer basics
  printf("\n\nPART 6 ----------\n");

  ip = a+1; // changed
  printf("\nstart %d", a[1]); // changed
  printf(" %d", *ip);
  printf(" %d", *(ip+1));
  printf(" %d", *ip++);
  printf(" %d", *ip);
  printf(" %d", *(ip+3));
  printf(" %d", *(ip-1));

  printf("\n\nThe integer pointer is %d bytes.", sizeof(ip));

  printf("\n\nWhen you print the pointer values themselves, you end up printing the address that the pointer is pointing to. You may want to use %%x instead of %%d because addresses in hex are easier to trace when you are debugging memory errors, or if you just want more readable address in general.");

  printf("\n\nThe difference between ip and ip+1 is not 1 because it depends on the system architecture. Since a 64-bit architecture is used here, the pointer is 8 bytes = 64 bits.");


  // 7 -- programming with pointers
  printf("\n\nPART 7 ----------\n");

  printf("\narray a: { ");
  i = 0;
  for (; i < 10; ++i) {
    printf("%d ", a[i]);
  }
  printf("}");

  i = 9;
  j = 0;
  while (j < 10) {
    b[j] = a[i];
    --i;
    ++j;
  }

  printf("\n\narray b: { ");
  i = 0;
  for (; i < 10; ++i) {
    printf("%d ", b[i]);
  }
  printf("}");

  ip = a;
  i = 0;
  for (; i < 10; ++i) {
    c[i] = *(ip+9-i);
  }

  printf("\n\narray c: { ");
  i = 0;
  for (; i < 10; ++i) {
    printf("%d ", c[i]);
  }
  printf("}");


  // 8 -- strings
  printf("\n\nPART 8 ----------\n");

  printf("\n%s\n", AString);
  i = 0;
  for (; i < 3; ++i) {
    printf("%d ", AString[i]);
  }

  printf("\n\n%d", AString[i]);
  printf("\nThe value of the byte after the last character is 0. This is the null byte/null-terminating byte/null char, which signifies the end of the string.");

  i = 0;
  for (; i < 3; ++i) {
    AString[i] += 1;
  }
  printf("\n\n%s", AString); // IBM!!!

  AString[i] += 60;
  printf("\n%s", AString);
  printf("\nAdding 60 to the byte following the last character in the string, the null byte, makes it no longer a null byte, and instead, the 'less than' character, '<'. Since it is no longer the null byte, the string will include the less than character. Also, since there is no longer a null byte, the string will continue until it hits a null (in the contiguous memory, as a string is just a character array, and arrays can be assumed to be contiguous).");


  // 9 -- address calculation
  printf("\n\nPART 9 ----------\n");

  printf("\n");
  for (k = 0; k < 10; k++) {
    printf("%x\n", &b[k]);
    b[k] = a[k];         // direct reference to array element
  }

  printf("\n");
  ip = a;
  ip2 = b;
  for (k = 0; k < 10; k++) {
    printf("%x\n", ip2);
    *ip2++ = *ip++;     // indirect reference to array element
  }


  // 10 -- programming practice/refresher
  printf("\nPART 10 ----------\n");
  int matrix[10][10];
  i = 0;
  j = 0;
  for (; i < 10; ++i) {
    for (; j < 10; ++j) {
      matrix[i][j] = i * 10 + j;
    }
    j = 0;
  }
  printf("\nOriginal matrix:\n");
  printMatrix(matrix);

  i = 0;
  j = i + 1;
  int temp;
  for (; i < 10; ++i) {
    for (; j < 10; ++j) {
      temp = matrix[i][j];
      matrix[i][j] = matrix[j][i];
      matrix[j][i] = temp;
    }
    j = i + 1;
  }
  printf("\nTransposed matrix:\n");
  printMatrix(matrix);


  // all done
  printf("\nALL DONE\n");
  return 0;
}
