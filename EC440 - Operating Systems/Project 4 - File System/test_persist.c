#include "fs.h"
#include "disk.h"
#include <assert.h>
#include <sys/wait.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#define MiB 1048576

const char *disk_name = "test_fs";
const char *file_name = "test_file";
char write_buf[BLOCK_SIZE];
char read_buf[sizeof(write_buf)];
int fd;

void child_process();
void parent_process(pid_t child);

int main() {
  srand(time(NULL));
    for (int ii = 0; ii < MiB; ++ii){
		write_buf[ii] = 'A' + rand() % 26;
  }

  pid_t pid = fork();
  if (pid == -1) {
    fprintf(stderr, "fork failed\n");
    exit(EXIT_FAILURE);
  } else if (pid == 0) {
    child_process();
  } else {
    parent_process(pid);
  }
}

void child_process() {
  assert(make_fs(disk_name) == 0);
  assert(mount_fs(disk_name) == 0);
  assert(fs_create(file_name) == 0);
  fd = fs_open(file_name);
  assert(fd >= 0);
  assert(fs_write(fd, write_buf, sizeof(write_buf)) == sizeof(write_buf));
  assert(fs_close(fd) == 0);
  assert(umount_fs(disk_name) == 0);
  exit(EXIT_SUCCESS);
}

void parent_process(pid_t child) {
  assert(waitpid(child, NULL, 0) == child);
  assert(mount_fs(disk_name) == 0);
  fd = fs_open(file_name);
  assert(fd >= 0);
  assert(fs_lseek(fd, 0) == 0);
  assert(fs_read(fd, read_buf, sizeof(read_buf)) == sizeof(read_buf));
  assert(strcmp(write_buf, read_buf) == 0);
  assert(fs_close(fd) == 0);
  assert(umount_fs(disk_name) == 0);
}
