override CFLAGS := -Wall -Werror -std=gnu99 -pedantic -O0 -g -pthread $(CFLAGS) #\
#       	-fsanitize=address -fsanitize=leak -fsanitize=undefined $(CFLAGS)
override LDLIBS := -pthread $(LDLIBS) #-fsanitize=address -fsanitize=leak -fsanitize=undefined $(LDLIBS)

tls.o: tls.c

#test_files = test_simple_create_destroy test_simple_read test_simple_write test_read_write \
	     test_simple_errors test_clone test_clone_read test_one_cow test_many_cow test_clone_large
#test_execs = $(addprefix ./, $(test_files))
#test_objects = $(addsuffix .o, $(test_files))

#all: check
#check: build
#	/bin/bash run_tests.sh $(test_execs)
#build: $(test_files)
#$(test_files): %: %.o tls.o
#$(test_objects): %.o: %.c

clean:
	rm -f *.o $(test_files)

.PHONY: all check build clean
