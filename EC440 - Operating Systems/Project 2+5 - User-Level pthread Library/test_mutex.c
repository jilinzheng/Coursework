#include <pthread.h>
#include <assert.h>
#include <stdio.h>

#define NUM_THREADS 5

int counter = 0;
pthread_mutex_t mutex;

// simple test routine that increments global variable counter by 1 million
void* routine(){
	for (int ii = 0; ii < 1000000; ++ii){
		pthread_mutex_lock(&mutex);
		++counter;
		pthread_mutex_unlock(&mutex);
	}
	return NULL;
}

int main (int argc, char *argv[]){
	// create five threads + init mutex
	pthread_t test_threads[NUM_THREADS];
	pthread_mutex_init(&mutex, NULL);

	// count/increment with NUM_THREADS threads
	for (int ii = 0; ii < NUM_THREADS; ++ii){
		pthread_create(test_threads+ii, NULL, &routine, NULL);
		printf("Thread %d is counting!\n", ii);
	}

	// join started threads
	for (int ii = 0; ii < NUM_THREADS; ++ii){
		pthread_join(*(test_threads+ii), NULL);
		printf("Thread %d has finished counting!\n", ii);
	}
	
	printf("Final value of counter: %d\n", counter);
	assert(counter == 5000000);
	pthread_mutex_destroy(&mutex);

	return 0;
}

