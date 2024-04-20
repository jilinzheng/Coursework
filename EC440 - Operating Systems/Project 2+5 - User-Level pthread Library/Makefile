.PHONY: clean checkprogs check 

override CFLAGS := -ggdb -std=gnu99 -O0 -Wall -pedantic $(CFLAGS) \

#test_files = test_busy_threads test_one_thread test_custom_schedule test_many_threads \
	     test_random_threads test_new_threads test_zombie_threads \
	     test_mutex test_critical_one test_critical_two test_barrier
test_files = test_mutex test_critical_one test_critical_two test_barrier test_manual_schedule
test_execs = $(addprefix ./, $(test_files))
test_objs = $(addsuffix .o, $(test_files))

all: submit 

submit: threads.o

# You might find it useful to write test programs and get them working against the installed pthreads
# to do so, set TSTMYPTHREADS to a value other than 1
TSTMYPTHREADS=1
ifeq ($(TSTMYPTHREADS),1)
	mythread = threads.o
else
	mythread =
	override LDFLAGS += -pthread
endif

threads.o: threads.c ec440threads.h

check: checkprogs
	/bin/bash run_tests.sh $(test_execs)

checkprogs: $(test_files)

$(test_files): %: %.o $(mythread)

clean:
	rm -f *.o $(test_files)

