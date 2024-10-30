#include "fs.h"
#include "disk.h"
#include <time.h>
#include <assert.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#define MiB 1048576

int main() {
    const char *disk_name = "test_fs";
    const char *file_name = "test_file";
    char write_buf[MiB];
    char read_buf[sizeof(write_buf)];
    char block_buf[BLOCK_SIZE];

    srand(time(NULL));
    for (int ii = 0; ii < MiB; ++ii){
		  write_buf[ii] = 'A' + rand() % 26;
    }

    assert(make_fs(disk_name) == 0);
    assert(mount_fs(disk_name) == 0);
    assert(fs_create(file_name) == 0);
    int fildes = fs_open(file_name);
    assert(fildes >= 0);

    assert(fs_write(fildes, write_buf, sizeof(write_buf)) == sizeof(write_buf));

    int ii = 4;
    int jj = 0;
    for ( ; ii < 260; ){
        //printf("ORIGINAL BLOCK %d:\n%.*s\n\n", ii, BLOCK_SIZE, write_buf+(jj*BLOCK_SIZE)); 
        if (ii > 15){
          block_read(ii, block_buf);
        } else { // to account for indirect block
          block_read(ii+1, block_buf);
        }
        //printf("WRITTEN BLOCK %d:\n%.*s\n\n", ii, BLOCK_SIZE, block_buf);
        assert(memcmp(block_buf, write_buf+(jj*BLOCK_SIZE), BLOCK_SIZE) == 0); // faulty atm
        ++ii;
        ++jj;
    }
   	
    assert(fs_get_filesize(fildes) == sizeof(write_buf));
    assert(fs_lseek(fildes, 0) == 0);
    assert(fs_read(fildes, read_buf, sizeof(read_buf)) == sizeof(read_buf));
    assert(memcmp(read_buf, write_buf, sizeof(read_buf)) == 0);

    assert(fs_close(fildes) == 0);
    assert(fs_delete(file_name) == 0);
    assert(umount_fs(disk_name) == 0);

    return 0;
}
