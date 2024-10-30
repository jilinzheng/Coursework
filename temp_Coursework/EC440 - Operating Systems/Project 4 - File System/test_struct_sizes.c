#include "fs.h"
#include "disk.h"

#include <stdint.h>
#include <limits.h>
#include <stdbool.h>
#include <sys/types.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

enum CONSTANTS {
	MAX_FILES = 64, 					// per assignment instruction
	MAX_FILENAME_LENGTH = 15, 			// also per assignment instruction
	MAX_INODE_DIRECT_OFFSETS = 12, 	// temporary just to see if 1 MiB files work
	MAX_OPEN_FILES = 32,
	SB_OFFSET = 0,
	INODE_OFFSET = 1,
	DIR_OFFSET = 2,
	BITMAP_OFFSET = 3,
	DATA_OFFSET = 4
};


/*--- STRUCT DECLARATIONS ---*/
struct superblock { // 5 * 2 bytes = 10 bytes
	uint16_t used_block_bitmap_count;
	uint16_t used_block_bitmap_offset;
	uint16_t inode_metadata_blocks;
	uint16_t inode_metadata_offset;
	uint16_t dir_block_offset;
};

struct inode { // 1 + 1 + (15 * 2) + 4 = 36 bytes
	bool is_used;
	uint16_t direct_offset[MAX_INODE_DIRECT_OFFSETS]; // up to 12 direct blocks per typical inode design
	uint16_t single_indirect_offset;
	uint16_t double_indirect_offset;
	uint16_t triple_indirect_offset;
	uint32_t file_size; // in bytes (a 30MiB file would be 31457280 bytes,
			    // which is too large for 2-byte(16 bits, uint16_t) storage; 2^16 = 65536
};

struct dentry { // 1 + 2 + 15 = 18 bytes
	bool is_used;
	uint16_t inode_number;
	char name[MAX_FILENAME_LENGTH]; // plus 1 for null-terminating byte ?
};

struct file_descriptor { // 1 + 2 + 2 = 5 bytes; in memory
	bool is_used;
	uint16_t inode_number;
	uint16_t offset;
};

int main(int argc, char *argv[]){
    struct inode inode_table[MAX_FILES];
    struct dentry root_dir[MAX_FILES];
    printf("sizeof struct superblock: %lu\n", sizeof(struct superblock));
    printf("sizeof struct inode: %lu\n", sizeof(struct inode));
    printf("sizeof inode_table with 64 elems: %lu\n", sizeof(inode_table));
    printf("sizeof struct dentry: %lu\n", sizeof(struct dentry));
    printf("sizeof root_dir with 64 elems: %lu\n", sizeof(root_dir));

    return 0;
}