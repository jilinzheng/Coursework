#include "fs.h"
#include "disk.h"

#include <stdint.h>
#include <limits.h>
#include <stdbool.h>
#include <sys/types.h>
#include <string.h>
#include <stdlib.h>

enum CONSTANTS {
	MAX_FILES = 64, 					// per assignment instruction
	MAX_FILENAME_LENGTH = 16, 			// ++1 for null-terminating byte
	MAX_INODE_DIRECT_OFFSETS = 12, 		// per typical inode implementation	
	MAX_OPEN_FILES = 32,
	SB_OFFSET = 0,
	INODE_OFFSET = 1,
	DIR_OFFSET = 2,
	BITMAP_OFFSET = 3,
	DATA_OFFSET = 4,
};


/*--- STRUCT DECLARATIONS ---*/
struct superblock { // 5 * 2 bytes = 10 bytes
	uint16_t inode_metadata_blocks;
	uint16_t inode_metadata_offset;
	uint16_t dir_block_offset;
	uint16_t used_block_bitmap_count;
	uint16_t used_block_bitmap_offset;
};

struct inode { // 1 + 1 + (15 * 2) + 4 = 36 bytes
	bool is_used;
	uint16_t direct_offset[MAX_INODE_DIRECT_OFFSETS]; // up to 12 direct blocks per typical inode design
	uint16_t single_indirect_offset; // points to data block of other data blocks' addresses
	uint32_t file_size;
};

struct dentry { // 20 bytes due to struct padding
	bool is_used;
	uint16_t inode_number;
	char name[MAX_FILENAME_LENGTH];
};

struct file_descriptor {
	bool is_used;
	uint16_t inode_number;
	uint32_t offset;
};


/*--- BITMAP OPERATIONS ---*/
static int get_bit(uint8_t *bitmap, int offset){
	return (((bitmap[offset/CHAR_BIT]) & (1 << (offset%CHAR_BIT))) >> (offset%CHAR_BIT)) & 1;
}

static void set_bit(uint8_t *bitmap, int offset){ // set bit to one
	bitmap[offset/CHAR_BIT] |= (uint8_t) 1 << (offset%CHAR_BIT);
}

static void clear_bit(uint8_t *bitmap, int offset){ // clear bit to zero
	bitmap[offset/CHAR_BIT] &= ~((uint8_t) 1 << (offset%CHAR_BIT));
}


/*--- GLOBALS ---*/
static struct superblock sb;
static struct inode inode_table[MAX_FILES];
static struct dentry root_dir[MAX_FILES];
static uint8_t used_block_bitmap[DISK_BLOCKS/CHAR_BIT];
static struct file_descriptor open_files[MAX_OPEN_FILES];


/*--- MANAGEMENT ROUTINES ---*/
int make_fs(const char *disk_name){
	if (make_disk(disk_name) != 0){
		return -1;
	}

	if (open_disk(disk_name) != 0){
		return -1;
	}

	// block buffer for block_read/block_write
	char block_buf[BLOCK_SIZE];
	memset(block_buf, '\0', BLOCK_SIZE);

	// initialize superblock and write out to disk
	struct superblock new_sb;
	new_sb.inode_metadata_blocks = 0;
	new_sb.inode_metadata_offset = INODE_OFFSET;
	new_sb.dir_block_offset = DIR_OFFSET;
	new_sb.used_block_bitmap_count = 0;
	new_sb.used_block_bitmap_offset = BITMAP_OFFSET;

	memcpy(block_buf, &new_sb, sizeof(struct superblock));
	if (block_write(SB_OFFSET, &block_buf) != 0){
		return -1;
	}
	memset(block_buf, '\0', BLOCK_SIZE);

	// initialize inode_table and write out to disk
	struct inode new_inode_table[MAX_FILES];
	for (int ii = 0; ii < MAX_FILES; ++ii){
		new_inode_table[ii].is_used = false;
		for (int jj = 0; jj < MAX_INODE_DIRECT_OFFSETS; ++jj){
			new_inode_table[ii].direct_offset[jj] = 0;
		}
		new_inode_table[ii].single_indirect_offset = 0;
		new_inode_table[ii].file_size = 0;
	}

	memcpy(block_buf, new_inode_table, MAX_FILES * sizeof(struct inode));
	if (block_write(INODE_OFFSET, &block_buf) != 0){
		return -1;
	}
	memset(block_buf, '\0', BLOCK_SIZE);

	// initialize directory and write out to disk
	struct dentry new_dir[MAX_FILES];
	for (int ii = 0; ii < MAX_FILES; ++ii){
		new_dir[ii].is_used = false;
		new_dir[ii].inode_number = ii;
		memset(new_dir[ii].name, '\0', MAX_FILENAME_LENGTH);
	}

	memcpy(block_buf, new_dir, MAX_FILES * sizeof(struct dentry));
	if (block_write(DIR_OFFSET, &block_buf) != 0){
		return -1;
	}
	memset(block_buf, '\0', BLOCK_SIZE);

	// initialize bitmap and write out to disk
	uint8_t new_used_block_bitmap[DISK_BLOCKS/CHAR_BIT];
	for(int ii = 0; ii < DISK_BLOCKS; ++ii){
		clear_bit(new_used_block_bitmap, ii);	
	}

	memcpy(block_buf, new_used_block_bitmap, sizeof(new_used_block_bitmap));
	if (block_write(BITMAP_OFFSET, &block_buf) != 0){
		return -1;
	}
	memset(block_buf, '\0', BLOCK_SIZE);

	if (close_disk() != 0){
		return -1;
	}

	return 0;
}

int mount_fs(const char *disk_name){
	if (open_disk(disk_name) != 0){
		return -1;
	}

	// block buffer for block_read/block_write
	char block_buf[BLOCK_SIZE];
	memset(block_buf, '\0', BLOCK_SIZE);

	// load superblock, inode table, directory and bitmap,
	// and initialize open_files array of fildes to no open files
	if (block_read(SB_OFFSET, block_buf) != 0){
		return -1;
	}
	memcpy(&sb, block_buf, sizeof(struct superblock));
	memset(block_buf, '\0', BLOCK_SIZE);
	
	if (block_read(INODE_OFFSET, block_buf) != 0){
		return -1;
	}
	memcpy(inode_table, block_buf, MAX_FILES * sizeof(struct inode));
	memset(block_buf, '\0', BLOCK_SIZE);

	if (block_read(DIR_OFFSET, block_buf) != 0){
		return -1;
	}
	memcpy(root_dir, block_buf, MAX_FILES * sizeof(struct dentry));
	memset(block_buf, '\0', BLOCK_SIZE);

	if (block_read(BITMAP_OFFSET, block_buf) != 0){
		return -1;
	}
	memcpy(used_block_bitmap, block_buf, sizeof(used_block_bitmap));
	memset(block_buf, '\0', BLOCK_SIZE);

	for (int ii = 0; ii < MAX_OPEN_FILES; ++ii){
		open_files[ii].is_used = false;
		open_files[ii].inode_number = 0;
		open_files[ii].offset = 0;
	}

	return 0;
}

int umount_fs(const char *disk_name){
	// block buffer for block_read/block_write
	char block_buf[BLOCK_SIZE];
	memset(block_buf, '\0', BLOCK_SIZE);

	// close all file descriptors
	for (int ii = 0; ii < MAX_OPEN_FILES; ++ii){
		fs_close(ii);
	}

	// use the block_buf and write the bitmap, directory, inode_table and superblock to disk
	memcpy(block_buf, used_block_bitmap, sizeof(used_block_bitmap));
	if (block_write(BITMAP_OFFSET, block_buf) != 0){
		return -1;
	}
	memset(block_buf, '\0', BLOCK_SIZE);

	memcpy(block_buf, root_dir, MAX_FILES * sizeof(struct dentry));
	if (block_write(DIR_OFFSET, block_buf) != 0){
		return -1;
	}
	memset(block_buf, '\0', BLOCK_SIZE);

	memcpy(block_buf, inode_table, MAX_FILES * sizeof(struct inode));
	if (block_write(INODE_OFFSET, block_buf) != 0){
		return -1;
	}
	memset(block_buf, '\0', BLOCK_SIZE);

	memcpy(block_buf, &sb, sizeof(struct superblock));
	if (block_write(SB_OFFSET, block_buf) != 0){
		return -1;
	}
	memset(block_buf, '\0', BLOCK_SIZE);

	if (close_disk() != 0){
		return -1;
	}

	return 0;
}


/*--- FILE SYSTEM FUNCTIONS ---*/
int fs_open(const char *name){
	// find the name in root_dir to verify file exists	
	int inode_number = 0;
	bool file_found = false;
	for (int ii = 0; ii < MAX_FILES; ++ii){
		if (root_dir[ii].is_used && strlen(root_dir[ii].name) != strlen(name)) continue;
		if (root_dir[ii].is_used && memcmp(root_dir[ii].name, name, strlen(name)) == 0){
			inode_number = root_dir[ii].inode_number;
			file_found = true;
			break;
		}
	}

	// return -1 error if file does not exist
	if (!file_found){
		return -1;	
	}

	// find the next free entry in open_files for the return file descriptor
	bool within_max_open_files = false;
	for (int ii = 0; ii < MAX_OPEN_FILES; ++ii){
		if (!open_files[ii].is_used){
			open_files[ii].is_used = true;
			open_files[ii].inode_number = inode_number;
			open_files[ii].offset = 0;
			within_max_open_files = true;	
			return ii;
		}	
	}

	// return -1 error if there are already 32 file descriptors active
	if (!within_max_open_files){
		return -1;
	}

	return -1; // should not reach here
}

int fs_close(int fildes){
	// return -1 error if fildes exceeds bounds of open_files array or fildes is not open/in use
	if (fildes < 0
		|| fildes >= MAX_OPEN_FILES
		|| open_files[fildes].is_used == false){
		return -1;
	}
	
	// reset the file_descriptor
	open_files[fildes].is_used = false;
	open_files[fildes].inode_number = 0;
	open_files[fildes].offset = 0;

	return 0;
}

int fs_create(const char *name){
	// return -1 error if name exceeds MAX_FILENAME_LENGTH or name is not given (<=0)
	if (strlen(name) > MAX_FILENAME_LENGTH || strlen(name) <= 0){
		return -1;
	}

	// return -1 error if name already exists as file
	for (int ii = 0; ii < MAX_FILES; ++ii){
		if (strlen(name) != strlen(root_dir[ii].name)) continue;
		if (memcmp(root_dir[ii].name, name, strlen(name)) == 0){
			return -1;
		}
	}

	// find the first empty inode and initialize
	bool empty_inode_found = false;
	uint16_t inode_number = 0;
	for (int ii = 0; ii < MAX_FILES; ++ii){
		if (!inode_table[ii].is_used){
			inode_table[ii].is_used = true;
			inode_table[ii].file_size = 0;
			empty_inode_found = true;
			inode_number = ii;
			++sb.inode_metadata_blocks;
			break;
		}
	}

	// return -1 error if there are no inodes left
	if (!empty_inode_found){
		return -1;
	}

	// find the first unused dentry, mark used and save filename
	bool empty_dentry_found = false;
	for (int ii = 0; ii < MAX_FILES; ++ii){
		// if we found an empty dentry
		if (!root_dir[ii].is_used){
			root_dir[ii].is_used = true;
			root_dir[ii].inode_number = inode_number;
			//memcpy(root_dir[ii].name, name, strlen(name));
			strcpy(root_dir[ii].name, name);
			empty_dentry_found = true;
			break;
		}
	}

	// return -1 error if root_dir is full
	if (!empty_dentry_found){
		return -1;
	}

	if (empty_inode_found && empty_dentry_found){
		return 0;
	}

	return -1; // should not reach here
}

int fs_delete(const char *name){
	// return -1 error when file does not exist or when file is currently open
	bool file_exists = false, file_in_use = false;
	struct inode *target_inode;
	struct dentry *target_dentry;
	for (int ii = 0; ii < MAX_FILES; ++ii){
		// find existing dentry for file
		if (memcmp(root_dir[ii].name, name, strlen(name)) == 0){
			file_exists = true;
			target_inode = &inode_table[root_dir[ii].inode_number];
			target_dentry = &root_dir[ii];
			// determine if file is open/in use
			for (int jj = 0; jj < MAX_OPEN_FILES; ++jj){
				if (open_files[jj].inode_number == target_dentry->inode_number
					&& open_files[jj].is_used){
					file_in_use = true;
					break;
				}
			}
			break;
		}
	}

	if (!file_exists || (file_exists && file_in_use)){
		return -1;
	}

	// remove inode for target file
	target_inode->is_used = false;
	
	// block_buf for writing null to blocks (delete blocks)
	char block_buf[BLOCK_SIZE];
	memset(block_buf, '\0', BLOCK_SIZE);
	for (int ii = 0; ii < MAX_INODE_DIRECT_OFFSETS; ++ii){
		if (target_inode->direct_offset[ii] != 0){
			if (block_write(target_inode->direct_offset[ii], block_buf) != 0){
				return -1;
			}
			clear_bit(used_block_bitmap, target_inode->direct_offset[ii]);
			target_inode->direct_offset[ii] = 0;
			--sb.used_block_bitmap_count;
		}
	}

	// if target inode has an indirect
	if (target_inode->single_indirect_offset != 0){
		block_read(target_inode->single_indirect_offset, block_buf);
		uint16_t block_to_clear;
		for (int ii = 0; ii < BLOCK_SIZE; ii+=2){
			memcpy(&block_to_clear, block_buf+ii, 2);
			clear_bit(used_block_bitmap, block_to_clear);
			--sb.used_block_bitmap_count;
		}
		memset(block_buf, '\0', BLOCK_SIZE);
		block_write(target_inode->single_indirect_offset, block_buf);
		clear_bit(used_block_bitmap, target_inode->single_indirect_offset);
		target_inode->single_indirect_offset = 0;
		--sb.used_block_bitmap_count;
	}

	target_inode->file_size = 0;

	// remove dentry for target file
	target_dentry->is_used = false;
	target_dentry->inode_number = 0;
	memset(target_dentry->name, '\0', MAX_FILENAME_LENGTH);
	--sb.inode_metadata_blocks;

	return 0;
}

static int get_smaller(size_t a, size_t b){
	return a <= b ? a : b;
}

int fs_read(int fildes, void *buf, size_t nbyte){
	if (fildes < 0 
		|| fildes >= MAX_OPEN_FILES
		|| open_files[fildes].is_used == false){
		return -1;
	}

	struct file_descriptor *target_file = &open_files[fildes];
	struct inode *target_inode = &inode_table[target_file->inode_number];
	char read_buf[BLOCK_SIZE];
	size_t bytes_read = 0;
	int ii = 0;						// to count up to nbyte
	int jj = target_file->offset;	// to traverse read_buf
	int bytes_end = 0;				// to get the smaller of nbyte vs. block_size - where the offset is (for block_read)

	// if the offset is at the end of the file
	if (target_file->offset >= target_inode->file_size) return 0;

	// offset is within the direct offsets
	for( ; ii < nbyte; ){
		// still within direct blocks
		if (jj/BLOCK_SIZE < MAX_INODE_DIRECT_OFFSETS){
			block_read(target_inode->direct_offset[jj/BLOCK_SIZE], read_buf);
			bytes_end = get_smaller(nbyte-ii, BLOCK_SIZE-(jj%BLOCK_SIZE));
			if (bytes_end != BLOCK_SIZE) bytes_end %= BLOCK_SIZE;
			memcpy((char *)buf+ii, read_buf+(jj%BLOCK_SIZE), bytes_end);
			ii+=bytes_end;
			jj+=bytes_end;
			bytes_read+=bytes_end;
			memset(read_buf, '\0', BLOCK_SIZE);
		}
		// now within indirect block
		else {
			char single_indirect_block_buf[BLOCK_SIZE];
			// read the entire single indirect block containing pointers to blocks
			block_read(target_inode->single_indirect_offset, single_indirect_block_buf);
			uint16_t block_to_read = 0;
			memcpy(&block_to_read, single_indirect_block_buf+(((jj/BLOCK_SIZE)-MAX_INODE_DIRECT_OFFSETS)*2), 2);
			block_read(block_to_read, read_buf); 
			bytes_end = get_smaller(nbyte-ii, BLOCK_SIZE-(jj%BLOCK_SIZE));
			if (bytes_end != BLOCK_SIZE) bytes_end %= BLOCK_SIZE;
			memcpy((char *)buf+ii, read_buf+(jj%BLOCK_SIZE), bytes_end);
			ii+=bytes_end;
			jj+=bytes_end;
			memset(read_buf, '\0', BLOCK_SIZE);
		}
	}

	uint32_t bytes_from_offset = target_inode->file_size - target_file->offset;
	bytes_read = ((bytes_from_offset < nbyte) ? bytes_from_offset : nbyte);
	target_file->offset += bytes_read;
	return bytes_read;
}

int fs_write(int fildes, void *buf, size_t nbyte){
	// return -1 error if fildes is out of bounds or not in use
	if (fildes < 0 
		|| fildes >= MAX_OPEN_FILES
		|| open_files[fildes].is_used == false){
		return -1;
	}

	struct file_descriptor *target_file = &open_files[fildes];
	struct inode *target_inode = &inode_table[target_file->inode_number];
	size_t bytes_written = 0;
	char write_buf[BLOCK_SIZE];
	int ii = 0;						// to traverse the buf
	int jj = target_file->offset;	// to not overwrite previously written data
	int bytes_end = 0;
	bool free_block_found = false;

	for ( ; ii < nbyte; ){
		// within direct blocks
		if (jj/BLOCK_SIZE < MAX_INODE_DIRECT_OFFSETS){
			// check if the offset has a block -> allocate/set reference to one if not
			if ((target_inode->direct_offset[jj/BLOCK_SIZE]) == 0){
				for (int kk = DATA_OFFSET; kk < DISK_BLOCKS; ++kk){
					if (get_bit(used_block_bitmap, kk) == 0){
						free_block_found = true;
						set_bit(used_block_bitmap, kk);
						++sb.used_block_bitmap_count;
						target_inode->direct_offset[jj/BLOCK_SIZE] = kk;
						break;
					}
				}
				if (!free_block_found){
					target_file->offset+=bytes_written;
					if (target_inode->file_size < jj) target_inode->file_size = jj;
					return 0;
				}
			}
			// read the appropriate block into write buf
			if (block_read(target_inode->direct_offset[jj/BLOCK_SIZE], write_buf) != 0){
				target_file->offset+=bytes_written;
				if (target_inode->file_size < jj) target_inode->file_size = jj;
				return bytes_written;
			}
			// perform write, increment tracking variables, and reset the write_buf afterwards
			bytes_end = get_smaller(nbyte-ii, BLOCK_SIZE-(jj%BLOCK_SIZE));
			if (bytes_end != BLOCK_SIZE) bytes_end %= BLOCK_SIZE;
			memcpy(write_buf+(jj%BLOCK_SIZE), (char*)buf+ii, bytes_end);
			if (block_write(target_inode->direct_offset[jj/BLOCK_SIZE], write_buf) != 0){
				target_file->offset+=bytes_written;
				if (target_inode->file_size < jj) target_inode->file_size = jj;
				return bytes_written;
			}
			ii+=bytes_end;
			jj+=bytes_end;
			bytes_written+=bytes_end;
			memset(write_buf, '0', BLOCK_SIZE);
		}
		// we are now within the indirect blocks
		else {
			char single_indirect_block_buf[BLOCK_SIZE];
			uint16_t block_to_write = 0;
			// verify the indirect block exists, if not, allocate and initialize it
			if (target_inode->single_indirect_offset == 0){
				for (int kk = DATA_OFFSET; kk < DISK_BLOCKS; ++kk){
					if (get_bit(used_block_bitmap, kk) == 0){
						free_block_found = true;
						set_bit(used_block_bitmap, kk);
						++sb.used_block_bitmap_count;
						memset(single_indirect_block_buf, '\0', BLOCK_SIZE);
						block_write(kk, single_indirect_block_buf);
						target_inode->single_indirect_offset = kk;
						break;
					}
				}	
				if (!free_block_found){
					target_file->offset+=bytes_written;
					if (target_inode->file_size < jj) target_inode->file_size = jj;
					return 0;
				}	
			}
			// read the indirect block into single indirect buf
			if (block_read(target_inode->single_indirect_offset, single_indirect_block_buf) != 0){
				target_file->offset+=bytes_written;
				if (target_inode->file_size < jj) target_inode->file_size = jj;
				return bytes_written;	
			}
			// 'index' into the single indirect buf and extract the appropriate block
			memcpy(&block_to_write, single_indirect_block_buf+(((jj/BLOCK_SIZE)-MAX_INODE_DIRECT_OFFSETS)*2), 2);
			// verify the indirect block's appropriate offset for our desired block is allocated, if not (block is zero), do so
			if (block_to_write == 0){
				// search for a block for the actual data, referenced through the single indirect
				for (int kk = DATA_OFFSET; kk < DISK_BLOCKS; ++kk){
					if (get_bit(used_block_bitmap, kk) == 0){
						free_block_found = true;
						set_bit(used_block_bitmap, kk);
						++sb.used_block_bitmap_count;
						memcpy(single_indirect_block_buf+(((jj/BLOCK_SIZE)-MAX_INODE_DIRECT_OFFSETS)*2), (uint16_t *)&kk, 2);
						block_write(target_inode->single_indirect_offset, single_indirect_block_buf);
						block_to_write = kk;
						break;
					}
				}	
				if (!free_block_found){
					target_file->offset+=bytes_written;
					if (target_inode->file_size < jj) target_inode->file_size = jj;
					return 0;
				}	
			}
			if (block_read(block_to_write, write_buf) != 0){
				target_file->offset+=bytes_written;
				if (target_inode->file_size < jj) target_inode->file_size = jj;
				return bytes_written;
			}
			// perform write, increment tracking variables, and reset the write_buf afterwards
			bytes_end = get_smaller(nbyte-ii, BLOCK_SIZE-(jj%BLOCK_SIZE));
			if (bytes_end != BLOCK_SIZE) bytes_end %= BLOCK_SIZE;
			memcpy(write_buf+(jj%BLOCK_SIZE), (char*)buf+ii, bytes_end);
			if (block_write(block_to_write, write_buf) != 0){
				target_file->offset+=bytes_written;
				if (target_inode->file_size < jj) target_inode->file_size = jj;
				return bytes_written;
			}
			ii+=bytes_end;
			jj+=bytes_end;
			bytes_written+=bytes_end;
			memset(write_buf, '0', BLOCK_SIZE);	
		}
	}

	// increment file pointer by bytes_written
	target_file->offset += bytes_written;
	if (target_inode->file_size < jj) target_inode->file_size = jj;
	return bytes_written;
}

int fs_get_filesize(int fildes){
	if (fildes < 0
		|| fildes >= MAX_OPEN_FILES
		|| open_files[fildes].is_used == false){
		return -1;
	}

	return inode_table[open_files[fildes].inode_number].file_size;
}

int fs_listfiles(char ***files){
	char **file_list = calloc(MAX_FILES+1, sizeof(char *));
	int file_count = 0;

	for (int ii = 0; ii < MAX_FILES; ++ii){
		if (root_dir[ii].is_used){
			file_list[file_count] = malloc(MAX_FILENAME_LENGTH+1);
			memcpy(file_list[file_count], root_dir[ii].name, MAX_FILENAME_LENGTH);
			memset(file_list[file_count]+MAX_FILENAME_LENGTH, '\0', 1);
			++file_count;
		}
	}

	file_list[file_count] = malloc(1);
	memset(file_list[file_count], '\0', 1);
	
	*files = file_list;

	return 0;
}

int fs_lseek(int fildes, off_t offset){
	if (fildes < 0
		|| fildes >= MAX_OPEN_FILES
		|| open_files[fildes].is_used == false
		|| offset < 0
		|| offset > inode_table[open_files[fildes].inode_number].file_size){
		return -1;
	}

	open_files[fildes].offset = offset;

	return 0;
}

int fs_truncate(int fildes, off_t length){
	if (fildes < 0
		|| fildes >= MAX_OPEN_FILES
		|| open_files[fildes].is_used == false
		|| length < 0
		|| length > inode_table[open_files[fildes].inode_number].file_size){
		return -1;
	}

	struct file_descriptor *target_file = &open_files[fildes];
	struct inode *target_inode = &inode_table[fildes];
	char block_buf[BLOCK_SIZE];

	for (int ii = target_inode->file_size; ii >= length; --ii){
		block_read(target_inode->direct_offset[ii/BLOCK_SIZE], block_buf);
		memset(block_buf+(ii%BLOCK_SIZE), '\0', 1);
		block_write(target_inode->direct_offset[ii/BLOCK_SIZE], block_buf);
	}

	target_inode->file_size = length;
	target_file->offset = length;

	return 0;
}
