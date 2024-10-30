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

void *worker(void *arg)
{
    int thread_id = (int)(long)arg;
    printf("Thread %d attempting to lock the mutex...\n", thread_id);
    pthread_mutex_lock(&mutex);
    printf("Thread %d has locked the mutex.\n", thread_id);

    // Simulate some work inside the critical section
    usleep(200000); // Sleep for 100 milliseconds to simulate work

    printf("Thread %d will now unlock the mutex.\n", thread_id);
    pthread_mutex_unlock(&mutex);

    pthread_exit(NULL);
    return NULL;
}

int main()
{
    pthread_t threads[NUM_THREADS];
    int i;

    // Initialize the mutex
    pthread_mutex_init(&mutex, NULL);

    // Create two threads
    for (i = 0; i < NUM_THREADS; i++)
    {
        if (pthread_create(&threads[i], NULL, worker, (void *)(long)i) != 0)
        {
            fprintf(stderr, "Failed to create thread %d\n", i);
            return 1;
        }
    }

    // Wait for both threads to complete
    for (i = 0; i < NUM_THREADS; i++)
    {
        pthread_join(threads[i], NULL);
    }

    // Clean up the mutex
    pthread_mutex_destroy(&mutex);

    return 0;
}