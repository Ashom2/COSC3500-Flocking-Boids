#include <iostream>
#include <math.h>
#include <list>
#include <immintrin.h>

struct Cell;

/**
* @brief updates all boids in cellsArr
*
* @param[in] cellsArr : array of cells used to divide simulation
* @return void
* */
void updateFrame(Cell cellsArr[]);