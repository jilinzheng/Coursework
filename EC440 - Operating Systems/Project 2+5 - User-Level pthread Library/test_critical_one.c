#include <pthread.h>
#include <stdlib.h>
#include <stdbool.h>
#include <setjmp.h>
#include <assert.h>
#include <stdio.h>
#include <unistd.h>
#include <signal.h>
#include "ec440threads.h"

#define THREAD_CNT 3

// Define a mutex
pthread_mutex_t mutex;

void *critical_section(void *arg)
{
    int my_num = (int)(long)arg;

    printf("Thread %d attempting to enter critical section...\n", my_num);
    pthread_mutex_lock(&mutex);
    printf("Thread %d has entered critical section.\n", my_num);

    // Simulate work by sleeping
    usleep(100000); // Sleep for 100 milliseconds

    printf("Thread %d is leaving critical section.\n", my_num);
    pthread_mutex_unlock(&mutex);
    printf("Thread %d has left critical section.\n", my_num);
    pthread_exit(NULL);
    printf("Thread %d has exited.\n", my_num);
    return NULL;
}

int main(int argc, char **argv)
{
    pthread_t threads[THREAD_CNT];
    int i;

    // Initialize mutex
    pthread_mutex_init(&mutex, NULL);

    // Create multiple threads
    for (i = 0; i < THREAD_CNT; i++)
    {
        pthread_create(&threads[i], NULL, critical_section, (void *)(long)i);
    }

    // Join threads
    for (i = 0; i < THREAD_CNT; i++)
    {
        pthread_join(threads[i], NULL);
    }

    // Destroy mutex
    pthread_mutex_destroy(&mutex);

    return 0;
}