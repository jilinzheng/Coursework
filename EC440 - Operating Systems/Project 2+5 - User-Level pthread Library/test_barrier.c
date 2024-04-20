#include <stdio.h>
#include <pthread.h>
#include "ec440threads.h"
#include <stdlib.h>
#include <signal.h>

#define THREAD_NUMS 4
pthread_barrier_t barrier;

void *t0(void *param)
{
    //schedule(SIGALRM);
    pthread_barrier_wait(&barrier);
    printf("t0 ready\n");
    return NULL;
}
void *t1(void *param)
{
    //schedule(SIGALRM);
    pthread_barrier_wait(&barrier);
    printf("t1 ready\n");
    return NULL;
}
void *t2(void *param)
{
    //schedule(SIGALRM);
    pthread_barrier_wait(&barrier);
    printf("t2 ready\n");
    return NULL;
}

int main(void)
{
    pthread_t t[3];

    assert(pthread_barrier_init(&barrier, NULL, THREAD_NUMS) == 0);

    pthread_create(&t[0], NULL, t0, NULL);
    //schedule(SIGALRM);
    pthread_create(&t[1], NULL, t1, NULL);
    //schedule(SIGALRM);
    pthread_create(&t[2], NULL, t2, NULL);
    //schedule(SIGALRM);

    pthread_barrier_wait(&barrier);
    //schedule(SIGALRM);
    //schedule(SIGALRM);
    //schedule(SIGALRM);
    printf("first iter: all sub threads ready, go!\n");

    for (int ii = 0; ii < THREAD_NUMS - 1; ++ii){
	pthread_join(t[ii], NULL);
    }

    assert(pthread_barrier_destroy(&barrier) == 0);

    assert(pthread_barrier_init(&barrier, NULL, THREAD_NUMS) == 0);

    pthread_create(&t[0], NULL, t0, NULL);
    //schedule(SIGALRM);
    pthread_create(&t[1], NULL, t1, NULL);
    //schedule(SIGALRM);
    pthread_create(&t[2], NULL, t2, NULL);
    //schedule(SIGALRM);

    pthread_barrier_wait(&barrier);
    //schedule(SIGALRM);
    //schedule(SIGALRM);
    //schedule(SIGALRM);
    printf("second iter! all sub threads ready, go!\n");

    for (int ii = 0; ii < THREAD_NUMS - 1; ++ii){
	pthread_join(t[ii], NULL);
    }

    assert(pthread_barrier_destroy(&barrier) == 0);

    return 0;
}
