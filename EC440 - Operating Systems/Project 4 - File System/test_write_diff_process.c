#include "fs.h"
#include "disk.h"
#include <time.h>
#include <assert.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#define MiB 1048576
#define TWENTY_BLOCKS 81920

int main() {
    const char *disk_name = "super_fs";
    const char *file_name = "super_file";
    char write_buf[TWENTY_BLOCKS];
    char block_buf[BLOCK_SIZE];

    for (int ii = 0; ii < TWENTY_BLOCKS; ii+=10){
        memcpy(write_buf+ii,"abcdefghij",10);
    }

    // fill write_buf with a random string of length MiB
    /*
    srand(time(NULL));
    for (int ii = 0; ii < MiB; ++ii){
		write_buf[ii] = 'A' + rand() % 26;
    }
    */
    
    // create and mount super_fs, create and open super_file
    assert(make_fs(disk_name) == 0);
    assert(mount_fs(disk_name) == 0);
    assert(fs_create(file_name) == 0);
    int fildes = fs_open(file_name);
    assert(fildes >= 0);
    
    // write the random write_buf to the super_file
    assert(fs_write(fildes, write_buf, sizeof(write_buf)) == sizeof(write_buf));
    block_read(4, block_buf);
    //printf("BLOCK 4 WRITTEN:\n");
    for (int ii = 0; ii < BLOCK_SIZE; ++ii){
        //printf("%c",block_buf[ii]);
    }
    //printf("\nEND\n");
    block_read(17, block_buf);
    //printf("BLOCK 17 (first indirect) WRITTEN:\n");
    for (int ii = 0; ii < BLOCK_SIZE; ++ii){
        //printf("%c",block_buf[ii]);
    }
    //printf("\nEND\n");

    assert(umount_fs(disk_name) == 0);

    return 0;
}
