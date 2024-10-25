#include <iostream>
#include <math.h>
#include <list>
#include <immintrin.h>

class Boid;

/*
Cell class to represent a subdivision of the simulation space containing a list of pointers to boids
*/
struct Cell {
    std::list<Boid*> boids;
};

/**
* @brief updates all boids in cellsArr
*
* @param[in] cellsArr : array of cells used to divide simulation
* @return void
* */
void updateFrame(Cell cellsArr[]);

Boid* initBoids(int numBoids);

Cell* initCells(int numBoids, Boid* arr, Cell* cellsArr);