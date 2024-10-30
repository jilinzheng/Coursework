#include <assert.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

/* How many threads (aside from main) to create */
#define THREAD_CNT 64

#define LIMIT (1 << 20)

// locations for return values
int some_value[THREAD_CNT];

void *count(void *arg) {
  int my_num = (long int)arg;
  int c = rand() % LIMIT;
  int i;

  for (i = 0; i < c; i++) {
    if ((i % 10000) == 0) {
      printf("id: 0x%lx num %d counted to %d of %d\n", pthread_self(), my_num,
             i, c);
    }
  }
  some_value[my_num] = c;

  pthread_exit(&some_value[my_num]);
  return NULL;
}

int main(int argc, char **argv) {
  pthread_t threads[THREAD_CNT];
  unsigned long int i;
  srand(time(NULL));

  for (i = 0; i < THREAD_CNT; i++) {
    pthread_create(&threads[i], NULL, count, (void *)i);
  }

  /* Collect statuses of the other threads, waiting for them to finish */
  for (i = 0; i < THREAD_CNT; i++) {
    void *pret;
    int ret;

    pthread_join(threads[i], &pret);
    assert(pret);
    ret = *(int *)pret;
    assert(ret == some_value[i]);
  }
  return 0;
}

