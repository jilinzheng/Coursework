#include "tls.h"
#include <pthread.h>
#include <signal.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>

#define MAX_THREADS 128
#define PAGE_SIZE 4096

/*
 * This is a good place to define any data structures you will use in this file.
 * For example:
 *  - struct TLS: may indicate information about a thread's local storage
 *    (which thread, how much storage, where is the storage in memory)
 *  - struct page: May indicate a shareable unit of memory (we specified in
 *    homework prompt that you don't need to offer fine-grain cloning and CoW,
 *    and that page granularity is sufficient). Relevant information for sharing
 *    could be: where is the shared page's data, and how many threads are sharing it
 *  - Some kind of data structure to help find a TLS, searching by thread ID.
 *    E.g., a list of thread IDs and their related TLS structs, or a hash table.
 */
typedef struct thread_local_storage {
	pthread_t tid;
	unsigned int size;		/* size in bytes */
	unsigned int page_num;		/* number of pages */
	struct page **pages;		/* array of pointer to pages */
} TLS;

struct page {
	size_t address;			/* start address of page */
	int ref_count;			/* counter for shared pages */
};

struct tid_tls_pair {
	pthread_t tid;
	TLS *tls;
};

/*
 * Now that data structures are defined, here's a good place to declare any
 * global variables.
 */
static struct tid_tls_pair *thread_storages[MAX_THREADS];
static bool pairs_initialized = false;
static bool handler_initialized = false;

/*
 * With global data declared, this is a good point to start defining your
 * static helper functions.
 */
// helper to initialize all thread_storage entries to NULL
static void init_pairs(){
	for (int ii = 0; ii < MAX_THREADS; ++ii){
		//thread_storages[ii]->tid = 0;
		//thread_storages[ii]->tls = NULL;
		thread_storages[ii] = NULL;
	}
}

static void tls_handle_page_fault(int sig, siginfo_t *si, void *context) {
	size_t p_fault = ((size_t) si->si_addr & ~(PAGE_SIZE - 1));

	if (p_fault) {
		pthread_exit(NULL);
	} else {
		signal(SIGSEGV, SIG_DFL);
		signal(SIGBUS, SIG_DFL);
		raise(sig);
	}
}

static void tls_init() {
	struct sigaction sigact;

	// handle page faults (SIGSEGV, SIGBUS)
	sigemptyset(&sigact.sa_mask);

	// give context to handler
	sigact.sa_flags = SA_SIGINFO;
	sigact.sa_sigaction = tls_handle_page_fault;
	sigaction(SIGBUS, &sigact, NULL);
	sigaction(SIGSEGV, &sigact, NULL);
}

static void tls_protect(struct page *p) {
	if (mprotect((void *) p->address, PAGE_SIZE, PROT_NONE)) {
		fprintf(stderr, "tls_protectL could not protect page\n");
		exit(1);
	}
}

static void tls_unprotect(struct page *p) {
	if (mprotect((void *) p->address, PAGE_SIZE, PROT_READ | PROT_WRITE)) {
		fprintf(stderr, "tls_unprotect: could not unprotect page\n");
		exit(1);
	}
}

/*
 * Lastly, here is a good place to add your externally-callable functions.
 */ 
int tls_create(unsigned int size)
{
	// error: size input is not valid
	if (size <= 0){
		return -1;
	}

    if (!handler_initialized){
        tls_init();
        handler_initialized = true;
    }

	// call helper to initialize all tid/tls pairs of global array if on first execution
	if (!pairs_initialized){
		init_pairs();
		pairs_initialized = true;
	}

	pthread_t curr_tid = pthread_self();		// get the current thread

	// error: the current thread already has a LSA
	for (int ii = 0; ii < MAX_THREADS; ++ii){
		if (thread_storages[ii] != NULL
			&& thread_storages[ii]->tid == curr_tid
			&& thread_storages[ii]->tls != NULL) {
			return -1;
		}
	}
	
	// allocate and initialize new tls
	TLS *tls = malloc(sizeof(TLS));
	tls->tid = curr_tid;
	tls->size = size;
	tls->page_num = size / PAGE_SIZE;
	if (size % PAGE_SIZE != 0){
		tls->page_num++;
	}	
	tls->pages = calloc(tls->page_num, sizeof(struct page *));
	for (int ii = 0; ii < tls->page_num; ++ii){
		tls->pages[ii] = malloc(sizeof(struct page));
		struct page *p = tls->pages[ii];
		p->address = (size_t) mmap(NULL, PAGE_SIZE, PROT_NONE, MAP_ANON | MAP_PRIVATE, 0, 0);
		p->ref_count = 1;
	}
	
	// add new tid/tls pair to the global array
	for (int ii = 0; ii < MAX_THREADS; ++ii){
		// use the first empty entry in the array to hold the tid_tls_pair
		if (thread_storages[ii] == NULL){
			thread_storages[ii] = malloc(sizeof(struct tid_tls_pair));
			thread_storages[ii]->tid = curr_tid;
			thread_storages[ii]->tls = tls;
			break;
		}
	}

	return 0;
}

int tls_destroy()
{
	pthread_t curr_tid = pthread_self();		// get current thread

	for (int ii = 0; ii < MAX_THREADS; ++ii){
		// error: the thread we want to destroy does not have a LSA
		if (thread_storages[ii] != NULL
			&& thread_storages[ii]->tid == curr_tid
			&& thread_storages[ii]->tls == NULL){
			return -1;
		}
		// this is the thread we want to destroy and it has a LSA
		if (thread_storages[ii] != NULL
			&& thread_storages[ii]->tid == curr_tid
			&& thread_storages[ii]->tls != NULL){
			// clean up all the pages
			TLS *curr_tls = thread_storages[ii]->tls;
			for (int jj = 0; jj < curr_tls->page_num; ++jj){
				// only free the page if it is not shared
				if (curr_tls->pages[jj]->ref_count == 1){
					if (munmap((void *)curr_tls->pages[jj]->address, PAGE_SIZE) != 0){
						perror("ERROR: ");
						return errno;
					}
					free(curr_tls->pages[jj]);	
				}
			        // else decrement the ref_count
				else {
					curr_tls->pages[jj]->ref_count--;
				}
			}
			free(curr_tls->pages);
			// clean up the tls and remove from global structure
			free(curr_tls);
			free(thread_storages[ii]);
			thread_storages[ii] = NULL;
			return 0;
		} 
	}	

	return -1; // we should never reach here
}

int tls_read(unsigned int offset, unsigned int length, char *buffer)
{
	pthread_t curr_tid = pthread_self();		// get current thread
	TLS *target_tls = NULL;			            // the TLS we shall read from

	// error handling
	for (int ii = 0; ii < MAX_THREADS; ++ii){
		// error: the thread was found in the pairs but it does NOT have a LSA
		if (thread_storages[ii] != NULL
			&& thread_storages[ii]->tid == curr_tid
			&& thread_storages[ii]->tls == NULL){
			return -1;
		}
		// error: the thread has a LSA but offset + length > size
		if (thread_storages[ii] != NULL
			&& thread_storages[ii]->tid == curr_tid
			&& thread_storages[ii]->tls != NULL
			&& offset + length > thread_storages[ii]->tls->size){
			return -1;
		}
		// we found the right thread and offset + length does not exceed size
		if (thread_storages[ii] != NULL
			&& thread_storages[ii]->tid == curr_tid
			&& thread_storages[ii]->tls != NULL
			&& offset + length <= thread_storages[ii]->tls->size){
			target_tls = thread_storages[ii]->tls;
			break;
		}
	}

	// if target_tls is still NULL, we did not find an appropriate tls
	if (target_tls == NULL){
		return -1;
	}

	// indices for unprotecting and reprotecting pages
	int orig_page = offset/PAGE_SIZE;
	int fin_page = -1;

	// perform read
	int ii = 0;
	int jj = offset;
	for ( ; jj < offset + length; ++ii, ++jj){
		int kk = jj / PAGE_SIZE; // which page to read from
		int ll = jj % PAGE_SIZE; // offset within the page to read from
		// unprotect page if not unprotected before
		if (kk != fin_page) {
			tls_unprotect(target_tls->pages[kk]);
			fin_page = kk;
		}
		unsigned char *p = (unsigned char *) target_tls->pages[kk]->address + ll;
		buffer[ii] = *p;
	}

	// reprotect unprotected pages
	for ( ; orig_page <= fin_page; ++orig_page){
		tls_protect(target_tls->pages[orig_page]);
	}

	return 0;
}

int tls_write(unsigned int offset, unsigned int length, const char *buffer)
{
	pthread_t curr_tid = pthread_self();		// get current thread
	TLS *target_tls = NULL;				// the TLS we shall write to

	// error handling
	for (int ii = 0; ii < MAX_THREADS; ++ii){
		// error: the thread was found in the pairs but it does NOT have a LSA
		if (thread_storages[ii] != NULL
			&& thread_storages[ii]->tid == curr_tid
			&& thread_storages[ii]->tls == NULL){
			return -1;
		}
		// error: the thread has a LSA but offset + length > size
		if (thread_storages[ii] != NULL
			&& thread_storages[ii]->tid == curr_tid
			&& thread_storages[ii]->tls != NULL
			&& offset + length > thread_storages[ii]->tls->size){
			return -1;
		}
		// we found the right thread and offset + length does not exceed size
		if (thread_storages[ii] != NULL
			&& thread_storages[ii]->tid == curr_tid
			&& thread_storages[ii]->tls != NULL
			&& offset + length <= thread_storages[ii]->tls->size){
			target_tls = thread_storages[ii]->tls;
			break;
		}
	}

	// if target_tls is still NULL, we did not find an appropriate tls
	if (target_tls == NULL){
		return -1;
	}

	// indices for unprotecting and reprotecting pages
	int orig_page = offset/PAGE_SIZE;
	int fin_page = -1;

	// perform write
	int ii = 0;
	int jj = offset;
	struct page *copy = NULL;
	for ( ; jj < offset + length; ++ii, ++jj){
		int kk = jj / PAGE_SIZE; 		// select appropriate page of target_tls
		int ll = jj % PAGE_SIZE;		// offset within page
		// unprotect page if not unprotected before
		if (kk != fin_page) {
			tls_unprotect(target_tls->pages[kk]);
			fin_page = kk;
		}

		// copy on write
		if (target_tls->pages[kk]->ref_count > 1){
			copy = (struct page *) calloc(1, sizeof(struct page));
			copy->address = (size_t) mmap(0, PAGE_SIZE, PROT_WRITE, MAP_ANON | MAP_PRIVATE, 0, 0);
			// memcpy the old page to the new page
			memcpy((void*) copy->address, (void *) target_tls->pages[kk]->address, PAGE_SIZE);
			// new page only has one reference
			copy->ref_count = 1;
			// decrement the old page's ref_count since curr thread is no longer using it
			target_tls->pages[kk]->ref_count--; 
			// reprotect the old page
			tls_protect(target_tls->pages[kk]);
			// set the tls' page pointer to the new copy made
			target_tls->pages[kk] = copy;
		}

		unsigned char *p = (unsigned char *) target_tls->pages[kk]->address + ll;
		*p = buffer[ii];
	}

	// reprotect unprotected pages
	for ( ; orig_page <= fin_page; ++orig_page){
		tls_protect(target_tls->pages[orig_page]);
	}

	return 0;
}

int tls_clone(pthread_t tid)
{
	pthread_t curr_tid = pthread_self();		// get current thread
	TLS *target_tls = NULL;				// the tls to clone from

	// error handling	
	for (int ii = 0; ii < MAX_THREADS; ++ii){
		// error: the thread was found in the pairs with a LSA
		if (thread_storages[ii] != NULL
			&& thread_storages[ii]->tid == curr_tid
			&& thread_storages[ii]->tls != NULL){
			return -1;
		}
		// error: the target thread to clone from has no LSA
		if (thread_storages[ii] != NULL
			&& thread_storages[ii]->tid == tid
			&& thread_storages[ii]->tls == NULL){
			return -1;
		}
		// found the target thread with a lsa
		if (thread_storages[ii] != NULL
			&& thread_storages[ii]->tid == tid
			&& thread_storages[ii]->tls != NULL){
			target_tls = thread_storages[ii]->tls;
		}
	}

	// if target_tls is still NULL, we did not find an appropriate tls
	if (target_tls == NULL){
		return -1;
	}

	// allocate and initialize new tls for curr thread
	TLS *tls = malloc(sizeof(TLS));
	tls->tid = curr_tid;
	tls->size = target_tls->size;
	tls->page_num = target_tls->page_num;
	tls->pages = calloc(tls->page_num, sizeof(struct page *));
	for (int ii = 0; ii < tls->page_num; ++ii){
		// point the new tls' pages to the same pages as target_tls' pages
		tls->pages[ii] = target_tls->pages[ii];
		tls->pages[ii]->ref_count++;
	}

	// add new tid/tls pair to the global array
	for (int ii = 0; ii < MAX_THREADS; ++ii){
		// use the first empty entry in the array to hold the tid_tls_pair
		if (thread_storages[ii] == NULL){
			thread_storages[ii] = malloc(sizeof(struct tid_tls_pair));
			thread_storages[ii]->tid = curr_tid;
			thread_storages[ii]->tls = tls;
			break;
		}
	}

	return 0;
}
