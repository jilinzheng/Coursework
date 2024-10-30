#include "fs.h"
#include "disk.h"
#include <assert.h>
#include <string.h>
#include <stdio.h>

int main() {
    const char *disk_name = "test_fs";
    const char *file_name = "test_file";
    char write_buf[4096];
    char read_buf[sizeof(write_buf)];
    char block_buf[4096];

    for (int ii = 0; ii < 4096; ++ii){
        write_buf[ii] = 'a';
    }

    assert(make_fs(disk_name) == 0);
    assert(mount_fs(disk_name) == 0);
    assert(fs_create(file_name) == 0);
    int fildes = fs_open(file_name);
    assert(fildes >= 0);
    assert(fs_write(fildes, write_buf, sizeof(write_buf)) == sizeof(write_buf));
    assert(fs_lseek(fildes, 0) == 0);
    assert(fs_read(fildes, read_buf, sizeof(read_buf)) == 4096);
    assert(memcmp(read_buf, write_buf, sizeof(read_buf)) == 0);
    assert(fs_get_filesize(fildes) == 4096);

    // Read bytes 501 - 510
    memset(read_buf, '\0',sizeof(read_buf));
    memcpy(write_buf, "helloworld", 10);
    assert(fs_lseek(fildes, 501) == 0);
    assert(fs_write(fildes, write_buf, 10) == 10);
    assert(fs_lseek(fildes, 501) == 0);
    assert(fs_read(fildes, read_buf, 10) == 10);
    assert(memcmp(read_buf, write_buf, 10) == 0);
    assert(fs_get_filesize(fildes) == 4096);

    // Read bytes 900 - 999
    memset(read_buf, '\0',sizeof(read_buf));
    for (int ii = 0; ii < 100; ++ii){
        write_buf[ii] = 'b';
    }
    assert(fs_lseek(fildes, 900) == 0);
    assert(fs_write(fildes, write_buf, 100) == 100);
    assert(fs_lseek(fildes, 900) == 0);
    assert(fs_read(fildes, read_buf, 100) == 100);
    assert(memcmp(read_buf, write_buf, 100) == 0);
    assert(fs_get_filesize(fildes) == 4096);

    // Read bytes 1000 - 1004
    memset(read_buf, '\0',sizeof(read_buf));
    for (int ii = 0; ii < 5; ++ii){
        write_buf[ii] = 'c';
    }
    assert(fs_lseek(fildes, 1000) == 0);
    assert(fs_write(fildes, write_buf, 5) == 5);
    assert(fs_lseek(fildes, 1000) == 0);
    assert(fs_read(fildes, read_buf, 5) == 5);
    assert(memcmp(read_buf, write_buf, 5) == 0);
    assert(fs_get_filesize(fildes) == 4096);
    block_read(4, block_buf);
    //printf("AFTER W/R BYTES 1000-1004:\n%.*s\n\n", BLOCK_SIZE, block_buf);

    // Append
    assert(fs_lseek(fildes, fs_get_filesize(fildes)) == 0);
    assert(fs_write(fildes, "super", 5) == 5);
    assert(fs_lseek(fildes, fs_get_filesize(fildes)-5) == 0);
    assert(fs_read(fildes, read_buf, 5) == 5);
    assert(memcmp(read_buf, "super", 5) == 0);
    assert(fs_get_filesize(fildes) == 4096+5);
    block_read(4, block_buf);
    //printf("AFTER APPENDING 'super':\n%.*s\n\n", BLOCK_SIZE, block_buf);

    // Append enough for a new block
    assert(fs_lseek(fildes, fs_get_filesize(fildes)) == 0);
    for (int ii = 0; ii < BLOCK_SIZE; ++ii){
        write_buf[ii] = 'p';
    }
    assert(fs_write(fildes, write_buf, BLOCK_SIZE) == BLOCK_SIZE);
    assert(fs_lseek(fildes, fs_get_filesize(fildes)-BLOCK_SIZE) == 0);
    assert(fs_read(fildes, read_buf, BLOCK_SIZE) == BLOCK_SIZE);
    assert(memcmp(read_buf, write_buf, BLOCK_SIZE) == 0);
    assert(fs_get_filesize(fildes) == 4096+5+BLOCK_SIZE);
    block_read(4, block_buf);
    //printf("AFTER APPENDING BLOCK OF 'p', BLOCK 4:\n%.*s\n\n", BLOCK_SIZE, block_buf);
    block_read(5, block_buf);
    //printf("AFTER APPENDING BLOCK OF 'p', BLOCK 5:\n%.*s\n\n", BLOCK_SIZE, block_buf);
    block_read(6, block_buf);
    //printf("AFTER APPENDING BLOCK OF 'p', BLOCK 6:\n%.*s\n\n", BLOCK_SIZE, block_buf);
    block_read(7, block_buf);
    //printf("AFTER APPENDING BLOCK OF 'p', BLOCK 7:\n%.*s\n\n", BLOCK_SIZE, block_buf);

    // Write a new file
    assert(fs_create("test_file2") == 0); // Create another file
    int fildes2 = fs_open("test_file2");
    assert(fildes2 >= 0);
    assert(fs_write(fildes2, "hello", 5) == 5);
    assert(fs_lseek(fildes2, 0) == 0);
    assert(fs_read(fildes2, read_buf, 5) == 5);
    assert(memcmp(read_buf, "hello", 5) == 0);
    assert(fs_get_filesize(fildes2) == 5);

    assert(fs_close(fildes2) == 0);
    assert(fs_close(fildes) == 0);
    assert(fs_delete(file_name) == 0);
    assert(umount_fs(disk_name) == 0);

    return 0;
}
