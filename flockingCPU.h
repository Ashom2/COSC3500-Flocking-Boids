#include <iostream>
#include <math.h>
#include <list>
#include <immintrin.h>

struct Cell;

class Boid;

/**
* @brief updates all boids in cellsArr
*
* @param[in] cellsArr : array of cells used to divide simulation
* @return void
* */
void updateFrame(Cell cellsArr[]);

Boid* initBoids(int numBoids);

Cell* initCells(Boid* arr, Cell* cellsArr);