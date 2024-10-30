#include "fs.h"
#include <assert.h>
#include <string.h>

int main() {
    const char *disk_name = "test_fs";
    const char *file_name = "test_file";
    char write_buf[8000];
    char read_buf[sizeof(write_buf)];

    for (int ii = 0; ii < 8000; ++ii){
        write_buf[ii] = 'a';
    }

    assert(make_fs(disk_name) == 0);
    assert(mount_fs(disk_name) == 0);
    assert(fs_create(file_name) == 0);
    int fildes = fs_open(file_name);
    assert(fildes >= 0);
    assert(fs_write(fildes, write_buf, sizeof(write_buf)) == sizeof(write_buf));
    assert(fs_lseek(fildes, 0) == 0);
    assert(fs_read(fildes, read_buf, sizeof(read_buf)) == sizeof(read_buf));
    assert(memcmp(read_buf, write_buf, sizeof(read_buf)) == 0);
    assert(fs_close(fildes) == 0);
    assert(fs_delete(file_name) == 0);
    assert(umount_fs(disk_name) == 0);

    return 0;
}
