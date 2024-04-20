# Jilin Zheng // U49258796 // File System

## Purpose

This project implemented an inode-based file system within a single file (a 'virtual disk'). The disk supports 8192 blocks, each being 4096 bytes (4 KiB). In my implementation, the 0th block is allocated for the superblock; 1st is inode table; 2nd is the directory; and 3rd is the used-blocks bitmap. To simplify the task, a number of simplifications were made by the instructors:

- Maximum file limit of 64 files;
- Maximum filename length of 15 characters;
- Maximum of 32 files open at one time
- A single root directory
- Single files will not exceed 1 MiB

This project was completed Spring 2024 for EC440 @ Boston University.
