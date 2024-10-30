/*
    Jilin Zheng
    jilin@bu.edu
*/

/*
Part A.
1. Determine the grid size, which in this case, is 10x10.
   Create a 10x10 (or other appropriate size) square grid of blanks/hyphens.
2. Beginning at the bottom left (0,0) (rows x columns), draw N+1, 2x1 vertical tiles of X's,
   with each new column beginning at one row above the previous column,
   i.e. if the input was N = 1, the first 2x1 X's will be at positions (0,0) and (1,0)
   and the second X's will be at positions (1,1) and (2,1) (one column over, started one row above).
   The X's should appear to form a staircase.
3. If N is even (including N = 0), draw an O one column to the right of the last X drawn.
   If N is odd, draw two O's, both two columns to the right of the last X column,
   with the first O aligned to the bottom X of the last X column,
   and the second O two tiles above the first O.
   For example, if N = 1, and the last X column occupied (1,1) and (2,1), the first O should be at (1,3),
   and the second O should be at (3,3).
4. If the X's and O's should exceed the predetermined grid size,
   discard the X's and O's that do not fit within the grid.
*/

#include <iostream>
#include <vector>
#include <cstdlib>

using namespace std;

void printGrid(vector<vector<char>> grid){
    for (int ii = 0; ii < 10; ii++){
        for (int jj = 0; jj < 10; jj++){
            cout << grid[ii][jj];
        }
        cout << endl;
    }
}

int main(int argc, char* argv[]){

    /* Create 10x10 grid of hyphens. */
    vector<vector<char>> grid(10, vector<char>(10, '-'));

    /* Save input number N to variable. */
    int N = atoi(argv[1]);

    if (N > 9 || N < 0) {
        cout << "Please enter a number between 0 and 9, inclusive." << endl;
        return 1;
    }

    /* Fill grid with appropriate X's. */
    for (int ii = 0; ii < N+1; ii++){
        grid[9-ii][ii] = 'x';
        if (ii == 9) continue;
        grid[8-ii][ii] = 'x';
    }

    /* Finish grid with appropriate O's. */
    if (N < 9){
        if (N % 2 == 0) {
            grid[7-N+1][N+1] = 'o';
        }
        else if (N < 8 && N % 2 != 0){
            grid[7-N][N+2] = 'o';
            grid[7-N+2][N+2] = 'o';
        }
    }

    /* Call to helper function to print the complete grid. */
    printGrid(grid);
    
    return 0;
}