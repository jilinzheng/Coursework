#include <string.h>
#include <stdlib.h>
#include <stdio.h>

int main(int argc, char* argv[]){
  char buffer[512] = {0};

  strncat(buffer, "apple", 1);
  strncat(buffer, "banana", 1);
  strncat(buffer, "citrus", 1);
  strncat(buffer, "dragonfruit", 1);
  strncat(buffer, "egg", 1);

  puts("Buffer currently stores: ");
  printf("%s", buffer);
  fflush(stdout);
  printf("\n%ld", strlen(buffer));

  buffer[strlen(buffer)-1] = '\0';
  printf("%s", buffer);
}
