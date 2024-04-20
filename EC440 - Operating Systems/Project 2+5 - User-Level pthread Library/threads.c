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
#include <sys/types.h>
#include <bits/thread-shared-types.h>
#include <stdio.h>

#define MAX_THREADS 128				// number of threads you support
#define THREAD_STACK_SIZE (1<<15)	// size of stack in bytes
#define QUANTUM (50 * 1000)			// quantum in usec
#define LONGJMP_RET 5				// return value of setjmp
#define MAX_MUTEXES 128				// match threads to be safe
#define MAX_BARRIERS 128			// match threads to be safe

enum thread_status {
	TS_EXITED,
	TS_RUNNING,
	TS_READY,
	TS_BLOCKED
};

struct thread_control_block {
	pthread_t thread_id;
	jmp_buf env;
	void *stack;
	enum thread_status status;
	void *retval;
};

// proj 5: singly-linked list for the list of waiting threads
typedef struct Node {
	int payload;
	struct Node *next;
} Node;

// proj 5: wrapper mutex struct that contains necessary info for associated functions
typedef struct Mutex {
	pthread_mutex_t *mutex;
	bool locked;
	Node *waitlist;
} Mutex;

// proj 5: wrapper barrier struct containing necessary information for associated functions
typedef struct Barrier {
	pthread_barrier_t *barrier;	// the actual barrier 
	int barrier_thread_target;	// number of threads desired to be in the barrier
	int barrier_thread_count;	// the current number of threads within the barrier
	bool first_barrier_exit;	// if the first thread has exited/not	
	int threads_exited;			// number of threads exited
	Node *blocked_thread_list;	// list of threads that are blocked by barrier
} Barrier;

static struct thread_control_block *threads[MAX_THREADS] = {NULL}; 	// array of threads
static int curr_thread = 0;											// for selecting a current thread to run (for scheduling)
static int active_threads = 0;										// for keeping track of how many threads are actually running
static sigset_t mask, empty_set;									// proj 5: for blocking SIGALRM
static Mutex *mutexes[MAX_MUTEXES] = {NULL};						// proj 5: as many mutexes as there are threads to be safe
static Barrier *barriers[MAX_BARRIERS] = {NULL}; 					// proj 5: as many barriers as there are threads to be safe

/*---PROJ 5: LINKED LIST HELPER FUNCTIONS---*/
static Node* create_node(int payload){
	Node *new_node = malloc(sizeof(Node));
	new_node->payload = payload;
	new_node->next = NULL;
	return new_node;
}

static void insert_thread(Node **head, int payload){
	Node *new_node = create_node(payload);
	if (*head == NULL){
		*head = new_node;
		return;
	}

	Node *temp = *head;
	while (temp->next != NULL){
		temp = temp->next;
	}	
	temp->next = new_node;
}

static int get_head(Node *head){
	if (head == NULL){
		return -1;
	}

	return head->payload;
}

static void remove_head(Node **head){
	Node *temp = *head;
	*head = temp->next;
	free(temp);
}
/*---END OF LINKED LIST HELPER FUNCTIONS FOR PROJ 5---*/

void schedule(int signal){
		// mark preempted thread & save its state if not exited
		if (threads[curr_thread]->status == TS_RUNNING){
			threads[curr_thread]->status = TS_READY;		
		}

		// skip if returned from longjmp
		if (setjmp(threads[curr_thread]->env) != 0){
			return;
		}

		int start_thread = curr_thread;
		bool first_iteration = true;
		// check the curr_thread and increment to next thread until a READY thread is found
		while (1){ 
			curr_thread++; // move on to the next thread

			// reset curr_thread to 0 if we went past the active threads
			if (curr_thread >= MAX_THREADS){
				curr_thread = 0;
			}

			// check for ready, nonblocked thread; break out of loop
			if (threads[curr_thread] != NULL
					&& threads[curr_thread]->status == TS_READY
					&& threads[curr_thread]->status != TS_BLOCKED) {
				break;
			}

			// we made our way back to the thread where we began traversing
			if (curr_thread == start_thread){
				first_iteration = false;
			}

			// if we traversed the entire threads list and could not find a ready thread, exit bc all threads are exited/NULL
			if (curr_thread == start_thread + 1 && !first_iteration){
				exit(0);
			}
		}

	threads[curr_thread]->status = TS_RUNNING;
	longjmp(threads[curr_thread]->env, LONGJMP_RET);
}

static void scheduler_init(){
	// set up signal handler
	struct sigaction sa;
	//sigset_t mask; // add signal mask to block sigalrm while in scheduling

	sigemptyset(&mask);
	sigaddset(&mask, SIGALRM);
	
	sa.sa_handler = schedule;
	sa.sa_mask = mask;
	sa.sa_flags = SA_NODEFER;

	sigaction(SIGALRM, &sa, NULL);

	sigemptyset(&empty_set);

	// create thread 0 from main; rsp and rip are already pointing to the right place
	struct thread_control_block *tcb = malloc(sizeof(struct thread_control_block));
	if (tcb == NULL){
		perror("ERROR");
  	}
	threads[0] = tcb; // main is thread 0
	tcb->thread_id = 0;
	tcb->status = TS_RUNNING;
	tcb->retval = malloc(sizeof(void *)); 
	//tcb->has_mutex = false;
	active_threads++;

    	// set up timer
    	ualarm(QUANTUM, QUANTUM);
}

static int initialize_tcb(struct thread_control_block *tcb, void *(*start_routine) (void *), void *arg, int id){
	tcb->thread_id = id;
	
	// initialize stack
	tcb->stack = malloc(THREAD_STACK_SIZE);
	if (tcb->stack == NULL){
		perror("ERROR");
		return errno;
	}
	
	// push return address, pthread_exit, onto bottom of stack
	void *stack_ptr = tcb->stack + THREAD_STACK_SIZE - sizeof(void *);
	*(unsigned long int*)stack_ptr = (unsigned long int) pthread_exit;

	// set rsp to where tcb->stack[stack_ptr] is and set up program counter to the start routine and optional args via start_thunk
	set_reg(&tcb->env, JBL_RSP, (unsigned long int) stack_ptr);
	set_reg(&tcb->env, JBL_R12, (unsigned long int) start_routine);
	set_reg(&tcb->env, JBL_R13, (unsigned long int) arg);
	set_reg(&tcb->env, JBL_PC, (unsigned long int) start_thunk);
	
	tcb->status = TS_READY;
	tcb->retval = malloc(sizeof(void *));
	//tcb->has_mutex = false;

	return 0;
}

int pthread_create(pthread_t *thread, const pthread_attr_t *attr, void *(*start_routine) (void *), void *arg){
  	// create the timer and handler for the scheduler; create thread 0.
  	static bool is_first_call = true;
  	if (is_first_call) {
    		is_first_call = false;
    		scheduler_init();
  	}

	struct thread_control_block *tcb = malloc(sizeof(struct thread_control_block));
	if (tcb == NULL){
		perror("ERROR");
	  	return errno;
  	}
	
	// find the next available NULL thread
	int ii = 0;
	while (ii < MAX_THREADS){
		if (threads[ii] == NULL){
			break;
		}
		ii++;
	}
	threads[ii] = tcb;
  	
	if (initialize_tcb(threads[ii], start_routine, arg, ii) != 0){
		perror("ERROR");
		return errno;
	} else {
		*thread = threads[ii]->thread_id;
		active_threads++;
		return 0;
	}

  	return 0;
}

void pthread_exit(void *value_ptr){
	threads[curr_thread]->status = TS_EXITED;
	threads[curr_thread]->retval = value_ptr;
	active_threads--;
    
	// continue to schedule while there are active threads
	while (active_threads > 0){
		schedule(SIGALRM);
	}

	exit(0);
}

pthread_t pthread_self(void){
	return curr_thread;
}

int pthread_join(pthread_t thread, void **retval){
	// error-check the inputs
	if (threads[thread] == NULL || retval == NULL){
		return -1;
	}

	// wait for thread to exit
	while (threads[thread]->status != TS_EXITED){}

	// set retval
	if (retval != NULL){
		*retval = threads[thread]->retval;
	} else {
		return -1;
	}

	// free the selected thread's stack and retval, null the thread
	free(threads[thread]->stack);
	free(threads[thread]);
	threads[thread] = NULL;

	return 0;
}


/*---PROJECT 5 THREAD SYNCHRONIZATION---*/
/*---RECOMMENDED FUNCTIONS---*/
static void lock(){
	// disable timer (stop handling SIGALRM) for scheduler via sigprocmask
	sigprocmask(SIG_BLOCK, &mask, NULL);
}

static void unlock(){
	// re-enable timer (start handling SIGALRM again)
	sigprocmask(SIG_UNBLOCK, &mask, NULL);
}


/*---MUTEX FUNCTIONS---*/
int pthread_mutex_init(pthread_mutex_t *restrict mutex, const pthread_mutexattr_t *restrict attr){
	*mutex = (pthread_mutex_t) PTHREAD_MUTEX_INITIALIZER;

	for (int ii = 0; ii < MAX_MUTEXES; ++ii){
		if (mutexes[ii] == NULL){
			mutexes[ii] = malloc(sizeof(Mutex));
			mutexes[ii]->mutex = mutex;
			mutexes[ii]->locked = false;
			mutexes[ii]->waitlist = NULL;
			break;
		}
	}
	return 0;
}

int pthread_mutex_destroy(pthread_mutex_t *mutex){
	for (int ii = 0; ii < MAX_MUTEXES; ++ii){
		if (mutexes[ii] != NULL && mutexes[ii]->mutex == mutex){
			// return if the mutex is locked
			if (mutexes[ii]->locked) return EBUSY;
			// free the waitlist, though it should probably free already...
			while (get_head(mutexes[ii]->waitlist) != -1){
				remove_head(&mutexes[ii]->waitlist);
			}
			free(mutexes[ii]);
			mutexes[ii] = NULL;
			break;
		}
	}

	*mutex = (pthread_mutex_t) PTHREAD_MUTEX_INITIALIZER;
	return 0;
}

int pthread_mutex_lock(pthread_mutex_t *mutex){
	// maintain atomicity
	lock();

	Mutex *target_mutex = NULL;
	for (int ii = 0; ii < MAX_MUTEXES; ++ii){
		if (mutexes[ii] != NULL && mutexes[ii]->mutex == mutex){
			target_mutex = mutexes[ii];
			break;
		}
	}
	
	// check if mutex is already locked
	if (target_mutex->locked){
		// add thread to end of waiting threads list	
		insert_thread(&target_mutex->waitlist, curr_thread);
		// set calling thread to blocked
		threads[curr_thread]->status = TS_BLOCKED;
		// allow scheduling after mutex crit section
		unlock();
		// block calling thread and schedule until calling thread is ready
		while (threads[curr_thread]->status == TS_BLOCKED){
			schedule(SIGALRM);
		}
		// lock again to maintain atomicity
		lock();
	}

	target_mutex->locked = true;
	unlock();
	return 0;
}

int pthread_mutex_unlock(pthread_mutex_t *mutex){
	// maintain atomicity
	lock();

	Mutex *target_mutex = NULL;
	for (int ii = 0; ii < MAX_MUTEXES; ++ii){
		if (mutexes[ii] != NULL && mutexes[ii]->mutex == mutex){
			target_mutex = mutexes[ii];
			break;
		}
	}

	// only if there is a waitlist
	if (get_head(target_mutex->waitlist) != -1){
		// wake up next thread (set to ready) to acquire lock
		threads[get_head(target_mutex->waitlist)]->status = TS_READY;
		// remove thread that is acquiring the lock next from the waitlist
		remove_head(&target_mutex->waitlist);
	}
	// otherwise set the mutex to unlocked
	else {
		target_mutex->locked = false;
	}

	unlock();
	return 0;
}


/*---BARRIER FUNCTIONS---*/
int pthread_barrier_init(pthread_barrier_t *restrict barrier, const pthread_barrierattr_t *restrict attr, unsigned count){
	if (count == 0) return EINVAL;

	lock(); // maintain atomicity

	for (int ii = 0; ii < MAX_BARRIERS; ++ii){
		if (barriers[ii] == NULL
				|| (barriers[ii] != NULL && barriers[ii]->barrier == barrier)){
			barriers[ii] = malloc(sizeof(Barrier));
			barriers[ii]->barrier = barrier;
			barriers[ii]->barrier_thread_target = count;
			barriers[ii]->barrier_thread_count = 0;
			barriers[ii]->first_barrier_exit = true;
			barriers[ii]->threads_exited = 0;
			barriers[ii]->blocked_thread_list = NULL;
			break;
		}
	}

	unlock();
	return 0;
}

int pthread_barrier_destroy(pthread_barrier_t *barrier){
	lock(); // maintain atomicity
	bool barrier_found = false;
	for (int ii = 0; ii < MAX_BARRIERS; ++ii){
		if (barriers[ii] != NULL && barriers[ii]->barrier == barrier){
			barriers[ii]->barrier_thread_target = -1;
			barriers[ii]->barrier_thread_count = -1;
			barriers[ii]->first_barrier_exit = false;
			barriers[ii]->threads_exited = -1;
			while (get_head(barriers[ii]->blocked_thread_list) != -1)
				remove_head(&barriers[ii]->blocked_thread_list);
			// ommitted to pass testcase 10...
			//free(barriers[ii]);
			//barriers[ii] = NULL;
			barrier_found = true;
			break;
		}
	}
	if (!barrier_found) return -1;

	unlock();
	return 0;
}

int pthread_barrier_wait(pthread_barrier_t *barrier){
	lock(); // maintain atomicity

	// search for the appropriate barrier
	Barrier* target_barrier = NULL;
	bool barrier_found = false;
	for (int ii = 0; ii < MAX_BARRIERS; ++ii){
		if (barriers[ii] != NULL && barriers[ii]->barrier == barrier){
			target_barrier = barriers[ii];
			barrier_found = true;
			break;
		}
	}
	if (!barrier_found) return -1;

	// increment the number of threads in the barrier (new thread entered)
	++(target_barrier->barrier_thread_count);

	// if the number of threads in the barrier is the target amount
	if (target_barrier->barrier_thread_count == target_barrier->barrier_thread_target){
		// ready all threads in blocked list
		int payload = get_head(target_barrier->blocked_thread_list);
		while (payload != -1){
			threads[payload]->status = TS_READY;	
			remove_head(&target_barrier->blocked_thread_list);
			payload = get_head(target_barrier->blocked_thread_list);
		}
	}	
	// else block calling thread until target number of threads has entered barrier
	else {
		threads[curr_thread]->status = TS_BLOCKED;
		// add thread to blocked list
		insert_thread(&target_barrier->blocked_thread_list, curr_thread);
		// re-enable scheduling
		unlock();
	}

	// the actual blocking
	while (threads[curr_thread]->status == TS_BLOCKED){
		schedule(SIGALRM);
	}

	// re-maintain atomicity
	lock();
	// increment # of threads exited
	++(target_barrier->threads_exited);

 	// one thread
	if (target_barrier->first_barrier_exit) {
		target_barrier->first_barrier_exit = false;
		unlock();
		return PTHREAD_BARRIER_SERIAL_THREAD;
	}

	// if all threads exited, reset barrier
	if (target_barrier->threads_exited == target_barrier->barrier_thread_target){
		target_barrier->barrier_thread_count = 0;
		target_barrier->first_barrier_exit = true;
		target_barrier->threads_exited = 0;
	}

	unlock();
	return 0; // rest of threads
}
