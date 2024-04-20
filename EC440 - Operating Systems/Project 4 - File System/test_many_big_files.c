#include "fs.h"
#include <assert.h>
#include <string.h>

int main() {
    const char *disk_name = "test_fs";
    const char *file_names[24] = {
        "1",  "2",  "3",  "4",  "5",  "6",  "7",  "8",  "9",  "10", "11",
        "12", "13", "14", "15", "16", "17", "18", "19", "20", "21", "22",
        "23", "24"
    };
    char write_buf[1048576];
    char read_buf[sizeof(write_buf)];

    for (int ii = 0; ii < 1048576; ++ii){
        write_buf[ii] = 'a';
    }

    assert(make_fs(disk_name) == 0);
    assert(mount_fs(disk_name) == 0);

    // create, write, read, and verify 24 1 MiB files
    for (int ii = 0; ii < 24; ++ii){
        assert(fs_create(file_names[ii]) == 0);
        int fildes = fs_open(file_names[ii]);
        assert(fildes >= 0);
        assert(fs_write(fildes, write_buf, sizeof(write_buf)) == sizeof(write_buf));
        assert(fs_lseek(fildes, 0) == 0);
        assert(fs_read(fildes, read_buf, sizeof(read_buf)) == sizeof(read_buf));
        assert(memcmp(read_buf, write_buf, sizeof(read_buf)) == 0);
        assert(fs_close(fildes) == 0);
    }
    
    // 10 x delete, recreate, delete 23 out of 24 files
    for (int jj = 0; jj < 10; ++jj){
        for (int ii = 1; ii < 24; ++ii){
            assert(fs_delete(file_names[ii]) == 0);
            assert(fs_create(file_names[ii]) == 0);
            int fildes = fs_open(file_names[ii]);
            assert(fildes >= 0);
            assert(fs_write(fildes, write_buf, sizeof(write_buf)) == sizeof(write_buf));
            assert(fs_close(fildes) == 0);
        }
    }

    // clean up all files
    for (int ii = 0; ii < 24; ++ii){
    	assert(fs_delete(file_names[ii]) == 0);
    }

    assert(umount_fs(disk_name) == 0);

    return 0;
}
