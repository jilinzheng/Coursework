# Jilin Zheng // U49258796 // User-Level pthread.h Library

## Purpose and Summary

April 17, 2024 Assignment 5 Update: Added pthread mutex and barrier functions using a linked list to complete assignment 5 thread synchronization. Included wrapper structs (`Mutex` and `Barrier`) around the `pthread_mutex_t` and `pthread_barrier_t` types, as well as a `Node` struct and helper functions for the linked list implementation (used for tracking blocked threads).

Assignment 2: `threads.c`, combined with the `ec440threads.h` header file, implements an user-level pthread.h library. The .c files preceded by `test_` have generously been provided by the EC440 course staff as well as an awesome anonymous classmate on Piazza. `fix.c` is a short program I wrote for my own debugging purposes. This assignment was completed Spring '24 for EC440 @ Boston University.

## Functions Breakdown

April 17, 2024 Assignement 5 Update:

- `pthread_mutex_init`: initializes a given mutex with the `PTHREAD_MUTEX_INITIALIZER` macro and adds the mutex to an arry of `Mutex` structs after initializing the wrapper `Mutex` struct
- `pthread_mutex_destroy`: resets the mutex to `PTHREAD_MUTEX_INITIALIZER` and frees it from the heap
- `pthread_mutex_lock`: allows a thread to lock an unlocked mutex, or block the thread until it acquires the mutex
- `pthread_mutex_unlock`: allows a thread to unlock a mutex, or if there are threads waiting to acquire the mutex, 'passes' the mutex to the first queued waiting thread by unblocking it
- `pthread_barrier_init`: initializes the wrapper around a barrier, `Barrier`, and adds it to an array of `Barrier` structs
- `pthread_barrier_destroy`: sets the targeted `Barrier` struct to invalid values
- `pthread_barrier_wait`: blocks the calling thread until the amount of threads to enter the barrier specified during `pthread_barrier_init` is reached, at which point all blocked threads are unblocked and may leave the barrier; one thread (the first to leave the barrier) gets a return value of the macro `PTHREAD_BARRIER_SERIAL_THREAD` while the rest get a return value of zero

Assignment 2:

- `schedule`: saves the current thread context and uses round-robin scheduling to find the next ready thread to longjmp to; called by a signal handler handling SIGALRM
- `scheduler_init`: initializes the signal handler handling SIGALRM, converts `main` into a thread with id 0 and status running, and sets up a timer that raises SIGALRM every `QUANTUM` (50000 microseconds)
- `initialize_tcb`: helper function for `pthread_create` that memory allocates a stack for a new thread, sets the new thread's registers, and sets its status to ready
- `pthread_create`: calls `scheduler_init` on first-time execution; memory allocates space for a `thread_control_block` and calls `initialize_tcb` to initialize the new thread being created
- `pthread_exit`: sets the calling thread status to exited and saves the exit status to the input `value_ptr`; calls schedule until there are no active threads left, and exits the program
- `pthread_self`: returns the id of the current thread
- `pthread_join`: ensures inputs are valid before waiting for the specified thread, then sets `retval` to the exit status previously passed to the `thread_control_block` upon `pthread_exit`; frees the previously memory-allocated stack and `thread_control_block` and sets the thread to `NULL` to be available for reuse
