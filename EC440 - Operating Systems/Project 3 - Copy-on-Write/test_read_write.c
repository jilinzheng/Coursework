#include "tls.h"
#include <assert.h>
#include <string.h>

#define TLS_SIZE 5

int main(int argc, char *argv[]){
	/*
	char str[5];
	str[0] = 'h';
	str[1] = 'e';
	str[2] = 'l';
	str[3] = 'l';
	str[4] = 'o';
	*/
	const char *str = "hello";
	assert(tls_create(TLS_SIZE) == 0);
	assert(tls_write(0, TLS_SIZE, str) == 0);

	char buf[TLS_SIZE];
	assert(tls_read(0, TLS_SIZE, buf) == 0);
	assert(strncmp(buf, str, TLS_SIZE) == 0);
	assert(tls_destroy() == 0);

	return 0;
}
