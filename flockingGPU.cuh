#include <stdio.h>
#include <stdarg.h>
#include <list>
#include <math.h>
#include <vector_types.h>//allows the use of composite types, e.g. float2, float3, int3, etc.

void setVars(int _xSize, int _ySize, int _marginSize, 
        float _turnFactor, 
        float _avoidRange, float _avoidFactor,
        float _visualRange, float _matchingFactor,
        float _cohesionFactor,
        float _minSpeed, float _maxSpeed,
        float _formationAngle);

void save(FILE *fptr, int numBoids, int frameNumber);

void init(int numBoids);

__host__ float updateBoids_GPU(int N);

void freeMemory();