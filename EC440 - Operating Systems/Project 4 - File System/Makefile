override CFLAGS := -g -std=gnu99 -O0 -Wall -pedantic -Werror $(CFLAGS) \
		-fsanitize=leak -fsanitize=address -fsanitize=undefined
override LDLIBS := -fsanitize=leak -fsanitize=address -fsanitize=undefined $(LDLIBS)

#test_files = test_management_routines test_fs_open test_mount_umount \
	     test_fs_create test_listfiles test_open_close test_fs_write \
	     test_get_filesize test_fs_read test_fs_delete test_truncate \
	     test_1k_read test_2k_read test_8k_read test_big_read_write \
	     test_many_big_files test_persist test_big_writes \
	     test_write_diff_process test_read_diff_process
test_files = test_write_diff_process test_read_diff_process
test_execs = $(addprefix ./, $(test_files))
test_objects = $(addsuffix .o, $(test_files))

#fs.o: fs.c disk.c

all: check

check: build
	/bin/bash run_tests.sh $(test_execs)

build: $(test_files)

$(test_files): %: %.o fs.o disk.o

$(test_objects): %.o: %.c

fs.o: fs.c disk.c

.PHONY: clean

clean:
	rm -f *.o $(test_files) test_disk fs_open_disk test_fs
