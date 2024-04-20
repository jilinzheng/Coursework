#include "fs.h"
#include <assert.h>

int main(int argc, char* argv[]){
    char *disk_name = "test_disk";
    assert(make_fs(disk_name) == 0);
    assert(mount_fs(disk_name) == 0);
    assert(umount_fs(disk_name) == 0);
    return 0;
}
