#include <stdio.h>
#include <chrono>

#include "flockingCPU.h"
#include "flockingGPU.cuh"

//Must be compiled with flockingCPU.cpp
//To run do:
//g++ -o flockingProfiler flockingProfiler.cpp flockingCPU.cpp
//Or
//Use the makefile / slurm script

const char *profilingFilepath = "ProfilingData.csv";

/*
Saves 
*/
void saveLine(FILE *fptr, int number, double time1, float time2, float time3) {
    // Write row to file
    fprintf(fptr, "%d, %g, %g, %g\n", number, time1, time2, time3);
}

//TODO check that the parameters are the same - especially numCells (or automate)
//TODO add error between comparisons
//TODO don't overwrite file, just add new line

//performance-affecting variables:
//number of boids
//Cell dimensions


//TODO these should be retrieved, or set the same
const int numCells_x = 16;
const int numCells_y = 16;

//For CPU
/*void ProfileAt(int N, FILE* fptr)
{
    //Initialize arrays
    Boid* boidsArray = initBoids(N);
    Cell cellsArray[numCells_x * numCells_y];
    initCells(N, boidsArray, cellsArray);    

    // Begin timing
    auto start = std::chrono::high_resolution_clock::now();

    // Do function
    updateFrame(cellsArray);

    // End timing
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::micro> duration_us = end - start;

    printf("N: %d. Time taken: %g us\n", N, duration_us.count());

    saveLine(fptr, N, duration_us.count());
}*/



//For GPU
void ProfileAt(int N, FILE* fptr)
{
    // Begin timing
    auto start = std::chrono::high_resolution_clock::now();

    //Initialize arrays
    setVars(512, 512, 64, 0.2, 8, 0.15, 20, 0.05, 0.2, 1, 2, 0.7 * 3.14);
    init(N);

    // End timing
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::micro> initDuration_us = end - start;

    //printf("N: %d. init time: %g us,     ", N, duration_us.count());


    
    // Begin timing
    start = std::chrono::high_resolution_clock::now();

    // Do function
    float kernelTime = updateBoids_GPU(N);

    // End timing
    end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::micro> duration_us = end - start;

    printf("N: %d. Time taken: %g us. Kernel time: %g us. CPU time: %g\n", N, duration_us.count(), kernelTime, duration_us.count() - kernelTime);

    saveLine(fptr, N, duration_us.count(), kernelTime, duration_us.count() - kernelTime);

    //Free the boids array memory
    freeMemory();
}



int main()
{
    // Create a file and open it for writing
    FILE *fptr;
    fptr = fopen(profilingFilepath, "w");
    if (fptr == NULL) {
        printf("%s", "Error opening file");
        return 1;
    }

    // Column headings
    fprintf(fptr, "N, Total time (us), Kernel time (us), CPU time (us)\n");

    

    //Logarithmic profiling
    
    /*int max = 1000000;
    for (int N = 1; N < max; N *= 10) {
        ProfileAt(N, fptr);
        ProfileAt(N * pow(10, 0.25), fptr);
        ProfileAt(N * pow(10, 0.5), fptr);
        ProfileAt(N * pow(10, 0.75), fptr);
    }
    ProfileAt(max, fptr);*/
    

    //Linear profiling
    int max = 1000000;
    int step = 5000;
    for (int N = step; N <= max; N += step) {
        ProfileAt(N, fptr);
    }

    // Close the file
    fclose(fptr);

    return 0;
}