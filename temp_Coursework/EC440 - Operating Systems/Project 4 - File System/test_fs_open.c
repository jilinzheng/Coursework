/*
 * Tests the functionality of fs_open and fs_close in fs.h
 */

#include <stdio.h>
#include <assert.h>
#include <string.h>
#include <stdlib.h>

#include "fs.h"

int main()
{
  printf("purposefully causing errors to check error catching\n\n");

  // create disk and mount
  assert(make_fs("fs_open_disk") == 0);
  assert(mount_fs("fs_open_disk") == 0);

  // test fs_open and fs_close and check all errors
  assert(fs_open("file0") == -1);
  assert(fs_create("temp") == 0);
  assert(fs_create("file0") == 0);

  int fdescr[32];
  for (int i = 0; i < 32; i++) {
    fdescr[i] = fs_open("file0");
    assert(fdescr[i] != -1);
  }

  assert(fs_open("file0") == -1);

  // close all fds
  for (int i = 0; i < 32; i++) {
    assert(fs_close(fdescr[i]) == 0);
  }

  // reopen all fds
  for (int i = 0; i < 32; i++) {
    fdescr[i] = fs_open("file0");
    assert(fdescr[i] != -1);
  }

  assert(fs_open("file0") == -1);
  
  // unmount to flush metadata
  assert(umount_fs("fs_open_disk") == 0);
	 
  return 0;
}
