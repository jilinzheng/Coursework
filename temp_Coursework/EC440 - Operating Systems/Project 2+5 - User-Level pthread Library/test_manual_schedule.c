#include <pthread.h>
#include <stdlib.h>
#include <stdbool.h>
#include <setjmp.h>
#include <assert.h>
#include <stdio.h>
#include <unistd.h>
#include <signal.h>
#include "ec440threads.h"

#define NUM_THREADS 2

pthread_mutex_t mutex;

void *worker1()
{
    printf("Thread 1 attempting to lock the mutex...\n");
    pthread_mutex_lock(&mutex);
    printf("Thread 1 has locked the mutex.\n");

    // Simulate some work inside the critical section
    usleep(200000); // Sleep for 200 milliseconds to simulate work

    printf("Thread 1 will now unlock the mutex.\n");
    pthread_mutex_unlock(&mutex);
    return NULL;
}

void *worker2()
{
    printf("Thread 2 attempting to lock the mutex...\n");
    schedule(0);
    pthread_mutex_lock(&mutex);
    printf("Thread 2 has locked the mutex.\n");

    // Simulate some work inside the critical section
    usleep(200000); // Sleep for 200 milliseconds to simulate work

    printf("Thread 2 will now unlock the mutex.\n");
    pthread_mutex_unlock(&mutex);
    return NULL;
}

int main()
{
    pthread_t threads[NUM_THREADS];
    int i;

    // Initialize the mutex
    pthread_mutex_init(&mutex, NULL);

    // Create two threads
    pthread_create(&threads[0], NULL, worker1, NULL);
    pthread_create(&threads[1], NULL, worker2, NULL);

    // Wait for both threads to complete
    for (i = 0; i < NUM_THREADS; i++)
    {
        pthread_join(threads[i], NULL);
    }

    // Clean up the mutex
    pthread_mutex_destroy(&mutex);

    return 0;
}
