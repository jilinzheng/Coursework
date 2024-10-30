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
    char read_buf[MiB];
    char block_buf[BLOCK_SIZE];
    char verify_buf[TWENTY_BLOCKS];

    for (int ii = 0; ii < TWENTY_BLOCKS; ii+=10){
        memcpy(verify_buf+ii,"abcdefghij",10);
    }

    // mount the EXISTING super_fs
    assert(mount_fs(disk_name) == 0);
    int fildes = fs_open(file_name);
    assert(fildes >= 0);

    // ensure the filesize is the MiB written
    /*
    assert(fs_get_filesize(fildes) == MiB);
    assert(fs_read(fildes, read_buf, MiB) == MiB);
    assert(memcmp(read_buf, verify_buf, MiB) == 0);
    */
    assert(fs_get_filesize(fildes) == TWENTY_BLOCKS);
    assert(fs_read(fildes, read_buf, TWENTY_BLOCKS) == TWENTY_BLOCKS);
    assert(memcmp(read_buf, verify_buf, TWENTY_BLOCKS) == 0);

    //printf("BLOCK 4 READ(via fs_read buf):\n");
    for (int ii = 0; ii < BLOCK_SIZE; ++ii){
        //printf("%c",read_buf[ii]);
    }
    //printf("\n");
    block_read(4, block_buf);
    //printf("\nBLOCK 4 READ(via block_read):\n");
    for (int ii = 0; ii < BLOCK_SIZE; ++ii){
	    //printf("%c",block_buf[ii]);
    }
    //printf("\n");

    //printf("BLOCK 17 (first indirect) READ(via fs_read buf):\n");
    for (int ii = 12*BLOCK_SIZE; ii < 13*BLOCK_SIZE; ++ii){ 	// 12 to index into the start of the indirect blocks in read_buf,
	   							// which is just contiguous data
        //printf("%c",read_buf[ii]);
    }
    //printf("\n");
    block_read(17, block_buf);
    //printf("\nBLOCK 17 (first indirect) READ(via block_read):\n");
    for (int ii = 0; ii < BLOCK_SIZE; ++ii){
	    //printf("%c",block_buf[ii]);
    }
    //printf("\n");

    // close the descriptor and delete the file, unmount the disk
    assert(fs_close(fildes) == 0);
    assert(fs_delete(file_name) == 0);
    assert(umount_fs(disk_name) == 0);

    return 0;
}
