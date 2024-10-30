#include <stdint.h>
#include <limits.h>
#include <stdbool.h>
#include <sys/types.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <assert.h>

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

int main(int argc, char *argv[]){
    static uint8_t bitmap[8192/CHAR_BIT];

    printf("The original bit: %d\n", get_bit(bitmap, 0));
    set_bit(bitmap, 0);
    printf("The new bit: %d\n", get_bit(bitmap, 0));
    clear_bit(bitmap, 0);
    printf("The cleared bit: %d\n\n", get_bit(bitmap, 0));

    printf("The original bit: %d\n", get_bit(bitmap, 5000));
    set_bit(bitmap, 5000);
    printf("The new bit: %d\n\n", get_bit(bitmap, 5000));
    clear_bit(bitmap, 5000);
    printf("The cleared bit: %d\n\n", get_bit(bitmap, 5000));

    printf("The original bit: %d\n", get_bit(bitmap, 1));
    set_bit(bitmap, 1);
    printf("The new bit: %x\n\n", get_bit(bitmap, 1));
    clear_bit(bitmap, 1);
    printf("The cleared bit: %d\n\n", get_bit(bitmap, 1));

    for (int ii = 0; ii < 8192; ++ii){
        set_bit(bitmap, ii);
        assert(get_bit(bitmap, ii) == 1);
        printf("SET OFFSET %d PASS.\n", ii);
        clear_bit(bitmap, ii);
        assert(get_bit(bitmap, ii) == 0);
        printf("CLEAR OFFSET %d PASS.\n\n", ii);
    }

    puts("BITMAP VERIFIED.\n");

    return 0;
}