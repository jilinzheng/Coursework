#include <pthread.h>
#include <stdlib.h>
#include <stdbool.h>
#include <setjmp.h>
#include <assert.h>
#include "ec440threads.h"
#include <errno.h>
#include <stdio.h>
#include <signal.h>
#include <sys/time.h>
#include <unistd.h>
#include <assert.h>

int retvals[5];

void *count(void *arg){
	int num = (long int) arg;
	int c = (num + 1) * 10000000;
	
	for (int ii = 0; ii < c; ii++){
		if ((ii % 10000) == 0){
			printf("id: 0x%lx num %d counted to %d of %d\n", pthread_self(), num, ii, c);
		}
	}

	retvals[num] = num;
	pthread_exit(&retvals[num]);

	return NULL;
}

int main(int argc, char* argv[]){
	// create a total of 5 + 1 (main) = 6 threads
	pthread_t t[5];

	unsigned long int ii;
	for (ii = 0; ii < 5; ii++){
		pthread_create(&t[ii], NULL, count, (void *) ii);
	}
	
	for (ii = 0; ii < 5; ii++){	
		void *retval;
		pthread_join(t[ii], &retval);
		assert(retvals[ii] == *(int *)retval);
	}

	printf("PASS\n");
	return 0;
}
