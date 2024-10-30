#include "flockingGPU.cuh"

#define BLOCKSIZE 512

const float PI = 3.141592653589793238462643383279502884;

const char *filepath = "data.txt";

int xSize = 512;
int ySize = 512;
__device__ int leftMargin = 64;
__device__ int rightMargin = 448;
__device__ int bottomMargin = 64;
__device__ int topMargin = 448;

// How hard the boid can turn to avoid walls
__device__ float turnFactor = 0.2;
// The distance within which separation occurs
float avoidRange = 8;
__device__ float sqrAvoidRange = 64;
// The rate at which separation occurs
__device__ float avoidFactor = 0.15;
// The distance within which alignment occurs
float visualRange = 20;
__device__ float sqrVisualRange = 400;
// The rate at which alignment occurs
__device__ float matchingFactor = 0.05;
// The rate at which cohesion occurs
__device__ float cohesionFactor = 0.2;
// The minimum speed of the boids
__device__ float minSpeed = 1;
// The maximum speed of the boids
__device__ float maxSpeed = 2;
// The formation angle
__device__ float formationAngle = 0.7 * PI;

// Calculate cell size
float cellSize = 32;
int numCells_x = 16;
int numCells_y = 16;
__device__ int deviceNumCells_x = 16;
__device__ int deviceNumCells_y = 16;
//const int cellSize = pow(2, ceil(log2(std::max(avoidRange, visualRange))));
// I failed to implement this using constexpr so you (the user) must compute manually







void setVars(int _xSize, int _ySize, int _marginSize, 
        float _turnFactor, 
        float _avoidRange, float _avoidFactor,
        float _visualRange, float _matchingFactor,
        float _cohesionFactor,
        float _minSpeed, float _maxSpeed,
        float _formationAngle)
{
    xSize = _xSize;
    ySize = _ySize;

    int _leftMargin = _marginSize;
    cudaMemcpyToSymbol(leftMargin, &_leftMargin, sizeof(int), 0, cudaMemcpyHostToDevice);
    int _rightMargin = _xSize - _marginSize;
    cudaMemcpyToSymbol(rightMargin, &_rightMargin, sizeof(int), 0, cudaMemcpyHostToDevice);
    int _bottomMargin = _marginSize;
    cudaMemcpyToSymbol(bottomMargin, &_bottomMargin, sizeof(int), 0, cudaMemcpyHostToDevice);
    int _topMargin = _ySize - _marginSize;
    cudaMemcpyToSymbol(topMargin, &_topMargin, sizeof(int), 0, cudaMemcpyHostToDevice);

    cudaMemcpyToSymbol(turnFactor, &_turnFactor, sizeof(float), 0, cudaMemcpyHostToDevice);

    float _sqrAvoidRange = _avoidRange * _avoidRange;
    cudaMemcpyToSymbol(sqrAvoidRange, &_sqrAvoidRange, sizeof(float), 0, cudaMemcpyHostToDevice);
    cudaMemcpyToSymbol(avoidFactor, &_avoidFactor, sizeof(float), 0, cudaMemcpyHostToDevice);

    float _sqrVisualRange = _visualRange * _visualRange;
    cudaMemcpyToSymbol(sqrVisualRange, &_sqrVisualRange, sizeof(float), 0, cudaMemcpyHostToDevice);
    cudaMemcpyToSymbol(matchingFactor, &_matchingFactor, sizeof(float), 0, cudaMemcpyHostToDevice);

    cudaMemcpyToSymbol(cohesionFactor, &_cohesionFactor, sizeof(float), 0, cudaMemcpyHostToDevice);

    cudaMemcpyToSymbol(minSpeed, &_minSpeed, sizeof(float), 0, cudaMemcpyHostToDevice);
    cudaMemcpyToSymbol(maxSpeed, &_maxSpeed, sizeof(float), 0, cudaMemcpyHostToDevice);

    cudaMemcpyToSymbol(formationAngle, &_formationAngle, sizeof(float), 0, cudaMemcpyHostToDevice);    
}



/*
Retrieves the x index of a cell at position x
*/
int getCell_x(float x) {
    int nx = (int)(x / cellSize);
    //clamp
    if (nx > numCells_x - 1) return numCells_x - 1;
    if (nx < 0) return 0;
    return nx;
}

/*
Retrieves the y index of a cell at position y
*/
int getCell_y(float y) {
    int ny = (int)(y / cellSize);
    //clamp
    if (ny > numCells_y - 1) return numCells_y - 1;
    if (ny < 0) return 0;
    return ny;
}

/*
Retrieves the 1D index of a cell at position x and y
*/
int getCell_i(float x, float y) {
    return getCell_x(x) + getCell_y(y) * numCells_x;
}



/*
Boid class to represent a single bird / particle / actor
*/
class Boid {
    public:
        float px;
        float py;
        float vx;
        float vy;
        uint cellIndex;
    
        Boid(float px, float py, float vx, float vy) {
            this->px = px;
            this->py = py;
            this->vx = vx;
            this->vy = vy;
            this->cellIndex = getCell_i(px, py);
        }
};

Boid* boidsArrayHost;
Boid** cellsArrayHost;
uint* cellOffsetsHost;
uint* cellSizesHost;



void freeMemory() {
    free(boidsArrayHost);
    for(int i = 0; i < numCells_x * numCells_y; i++) free(cellsArrayHost[i]);
    free(cellsArrayHost);
    free(cellOffsetsHost);
    free(cellSizesHost);
}

/*
Saves 
*/
void save(FILE *fptr, int numBoids, int frameNumber) {
    // Write vector array to file
    fprintf(fptr, "Frame %d\n", frameNumber);
    for(int i = 0; i < numBoids; i++) {
        fprintf(fptr, "%f %f %f %f\n", boidsArrayHost[i].px, boidsArrayHost[i].py, boidsArrayHost[i].vx, boidsArrayHost[i].vy);
    }    
}



// Sourced from https://stackoverflow.com/questions/686353/random-float-number-generation
float randFloat(float min, float max) {
    return min + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(max-min)));
}



/*
Magnitude of a vector
*/
__device__ float mag(float x, float y) {
    return sqrt(x * x + y * y);
}



/*
Square magnitude of a vector
*/
__device__ float sqrMag(float x, float y) {
    return x * x + y * y;
}



/*
Vector dot product
*/
__device__ float dot(float x1, float y1, float x2, float y2) {
    return x1 * x2 + y1 * y2;
}



/*
Gets the orthogonal vector using pass-by-reference
*/
__device__ void getOrthogonal(float &orthogonalVector_x, float &orthogonalVector_y, 
        float diffVector_x, float diffVector_y, 
        float formationDir_x, float formationDir_y) {
    //TODO expand and simplify    
    // Determines whether formation is to the left or right of boid using cross product
    // And constructs formation vector accordingly by rotating formationDir
    float formationVector_x, formationVector_y;
    float c = cos(formationAngle);
    float s = sin(formationAngle);
    if (formationDir_x * diffVector_y - formationDir_y * diffVector_x > 0) { //If boid is to the left of formation
        formationVector_x = formationDir_x * c - formationDir_y * s;
        formationVector_y = formationDir_x * s - formationDir_y * c;
    }
    else { //If boid is to the right of formation
        formationVector_x = formationDir_x * c + formationDir_y * s;
        formationVector_y = formationDir_y * c - formationDir_x * s;
    }

    // Check that formation is ahead of boid not behind (the dot product is more than 90 degrees)
    // This is to stop the leaders from trying to fall in line behind
    if (dot(formationDir_x, formationDir_y, diffVector_x, diffVector_y) < 0) {
        // Get at the vector orthogonal to the formationVector and move in that direction
        float sqrM = sqrMag(formationVector_x, formationVector_y);
        float val = dot(diffVector_x, diffVector_y, formationVector_x, formationVector_y) / sqrM;
        orthogonalVector_x = val * formationVector_x - diffVector_x;
        orthogonalVector_y = val * formationVector_y - diffVector_y;
    }
}



//Using shared memory
__global__ void updateBoidsKernel_GPU(int N, Boid* cellsArray, uint* cellOffsetsArray, uint* cellSizesArray, Boid* out)
{
    //This boid's index
    int thisIndex = blockIdx.x * blockDim.x + threadIdx.x;

    if (thisIndex < N) { //Check out of bounds
        float avoidVector_x = 0, avoidVector_y = 0;
        float formationDir_x = 0, formationDir_y = 0;
        float formationPos_x = 0, formationPos_y = 0;
        int neighboringBoids = 0;

        //For some reason Boid& o = cellsArray[otherIndex]; seems to make it faster but making this line a reference makes it slower
        Boid b = cellsArray[thisIndex];

        //TODO there must be a more efficient way of doing this
        int thisCellIndex = b.cellIndex;
        int cell_x = thisCellIndex % deviceNumCells_x;
        int cell_y = thisCellIndex / deviceNumCells_x;
        // Iterate over neighboring cells
        for(int x = cell_x - 1; x <= cell_x + 1; x++) {
            if (x < 0 || x >= deviceNumCells_x) continue; //Ignore cells beyond boundary
            for(int y = cell_y - 1; y <= cell_y + 1; y++) {
                if (y < 0 || y >= deviceNumCells_y) continue; //Ignore cells beyond boundary

                uint otherCellIndex = x + y * deviceNumCells_x;

                //Iterate over each boid in neighboring cell
                for (int otherIndex = cellOffsetsArray[otherCellIndex]; otherIndex < cellOffsetsArray[otherCellIndex] + cellSizesArray[otherCellIndex]; otherIndex++) {

                    Boid& o = cellsArray[otherIndex];

                    if (&o == &cellsArray[thisIndex]) continue; //Ignore itself

                    //printf("          This cell (%d, %d): Found boid (%.0f, %.0f). in cell: %d, %d\n", cell_x, cell_y, o.px, o.py, x, y);

                    // Get the distance between this boid and other boid
                    float sqrDist = sqrMag(b.px - o.px, b.py - o.py);
                    if (sqrDist < sqrAvoidRange) { // If the distance is less than protected range
                        //Divide by the square of distance to make avoidance exponential and smoother
                        avoidVector_x += (b.px - o.px) / sqrDist;
                        avoidVector_y += (b.py - o.py) / sqrDist;
                    }
                    if (sqrDist < sqrVisualRange) { // If the distance is less than visual range
                        formationDir_x += o.vx;
                        formationDir_y += o.vy;
                        formationPos_x += o.px;
                        formationPos_y += o.py;
                        neighboringBoids++;
                    }
                }
            }
        }

        //Make a copy for out
        Boid& bOut = out[thisIndex]; //Reference to out[thisIndex]
        bOut = b;

        // Separation - move away from nearby boids
        bOut.vx += avoidVector_x * avoidFactor;
        bOut.vy += avoidVector_y * avoidFactor;

        if (neighboringBoids > 0) { //If there were any boids in visual range
            // Get mean formation direction and position
            formationDir_x /= neighboringBoids;
            formationDir_y /= neighboringBoids;
            formationPos_x /= neighboringBoids;
            formationPos_y /= neighboringBoids;  

            // Alignment - match the mean velocity of all boids in visual range
            bOut.vx += (formationDir_x - bOut.vx) * matchingFactor;
            bOut.vy += (formationDir_y - bOut.vy) * matchingFactor;

            // Flocking
            // Represents a vector pointed dowards this boid from the centre of mass
            float diffVector_x = bOut.px - formationPos_x;
            float diffVector_y = bOut.py - formationPos_y;

            float orthogonalVector_x = 0, orthogonalVector_y = 0;
            getOrthogonal(orthogonalVector_x, orthogonalVector_y, 
                    diffVector_x, diffVector_y,
                    formationDir_x, formationDir_y);

            bOut.vx += orthogonalVector_x * cohesionFactor;
            bOut.vy += orthogonalVector_y * cohesionFactor;            
        }



        // Avoid edges
        if (bOut.px < leftMargin) {
            bOut.vx += turnFactor;
        }
        else if (bOut.px > rightMargin) {
            bOut.vx -= turnFactor;
        }
        if (bOut.py < bottomMargin) {
            bOut.vy += turnFactor;
        }
        else if (bOut.py > topMargin) {
            bOut.vy -= turnFactor;
        }
        //---------------------------------------



        // Impose speed limit on boid
        float speed = mag(bOut.vx, bOut.vy);
        if (speed > maxSpeed) {
            bOut.vx *= maxSpeed / speed;
            bOut.vy *= maxSpeed / speed;
        }
        else if (speed < minSpeed) {
            bOut.vx *= minSpeed / speed;
            bOut.vy *= minSpeed / speed;
        }
        //else if (speed == 0) {} // TODO
        //---------------------------------------

        
        
        // Update boid position
        bOut.px += bOut.vx;
        bOut.py += bOut.vy;
        //---------------------------------------
    }
}



void assignBoidsToCells(int numBoids)
{
    //Reset cells
    for(int i = 0; i < numCells_x * numCells_y; i++) {
        cellsArrayHost[i] = (Boid*)malloc(0);
        cellSizesHost[i] = 0;
        cellOffsetsHost[i] = 0;
    }

    //Determine cell sizes
    for(int i = 0; i < numBoids; i++) {
        Boid& b = boidsArrayHost[i];
        b.cellIndex = getCell_i(b.px, b.py); //TODO dont calculate this here
        //increase cell size
        cellSizesHost[b.cellIndex]++;
    }

    //Reallocate memory
    for(int i = 0; i < numCells_x * numCells_y; i++) {
        cellsArrayHost[i] = (Boid*)realloc(cellsArrayHost[i], cellSizesHost[i] * sizeof(Boid));
        cellSizesHost[i] = 0; //Reset to 0 so we can use as an incrementer
    }

    //Put boids in cells
    for(int i = 0; i < numBoids; i++) {
        Boid& b = boidsArrayHost[i];
        //Allocate boid to a cell in cellsArrayHost
        cellsArrayHost[b.cellIndex][cellSizesHost[b.cellIndex]++] = b;
    }
}



__host__ float updateBoids_GPU(int N)
{
    size_t size1 = N * sizeof(Boid);
    size_t size2 = numCells_x * numCells_y * sizeof(uint);
    
    auto start = std::chrono::high_resolution_clock::now();

    //Recalculate which cells have which boids
    assignBoidsToCells(N);

    

    //flatten cells array so we can put it onto the kernel
    Boid* flattenedCellsArray = (Boid*)malloc(size1);
    int indexOffset = 0;
    for (int i = 0; i < numCells_x * numCells_y; i++) {
        cellOffsetsHost[i] = indexOffset;
        for (int j = 0; j < cellSizesHost[i]; j++) {
            flattenedCellsArray[indexOffset + j] = cellsArrayHost[i][j];
        }
        indexOffset += cellSizesHost[i];
    }


    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::micro> duration_us = end - start;
    printf("N: %d. ------------- CPU Time taken: %g us.\n", N, duration_us.count());

    //Allocate memory on the device
    Boid* deviceCells;
    uint *deviceCellOffsets;
    uint *deviceCellSizes;
    Boid *deviceOut;
    cudaMalloc(&deviceCells, size1);
    cudaMalloc(&deviceCellOffsets, size2);
    cudaMalloc(&deviceCellSizes, size2);
    cudaMalloc(&deviceOut, size1);

    //Copy memory from host to device
    cudaMemcpy(deviceCells, flattenedCellsArray, size1, cudaMemcpyHostToDevice);
    cudaMemcpy(deviceCellOffsets, cellOffsetsHost, size2, cudaMemcpyHostToDevice);
    cudaMemcpy(deviceCellSizes, cellSizesHost, size2, cudaMemcpyHostToDevice);
    //printf("Device Variable Copying:\t%s\n", cudaGetErrorString(cudaGetLastError()));

    //Specify blocks and threads, using 1D threads and blocks since our data is 1D
    int threadsPerBlock = BLOCKSIZE;
    int numBlocks = (N + threadsPerBlock - 1) / threadsPerBlock; //Int division of N / threadsPerBlock

    //Start Timer
    cudaEvent_t startGPU;
    cudaEvent_t stopGPU;
    cudaEventCreate(&startGPU);
    cudaEventCreate(&stopGPU);
    cudaEventRecord(startGPU);
    
    //Run
    updateBoidsKernel_GPU<<<numBlocks, threadsPerBlock>>>(N, deviceCells, deviceCellOffsets, deviceCellSizes, deviceOut);

    //Mark end of kernel execution
    cudaEventRecord(stopGPU);
    //Wait for results to be available
    cudaEventSynchronize(stopGPU);   
    //Measure the time the kernel took to execute
    float kernelTime;
    cudaEventElapsedTime(&kernelTime, startGPU, stopGPU);

    //Wait for the kernel to synchronise
    cudaDeviceSynchronize();

    //Copy memory from device to host
    cudaMemcpy(boidsArrayHost, deviceOut, size1, cudaMemcpyDeviceToHost);

    //Clean up
    cudaFree(deviceCells);
    cudaFree(deviceCellOffsets);
    cudaFree(deviceCellSizes);
    cudaFree(deviceOut);

    return kernelTime * 1000;
}



void init(int numBoids)
{
    //Malloc boids
    boidsArrayHost = (Boid*)malloc(numBoids * sizeof(Boid));

    //Malloc cells
    cellsArrayHost = (Boid**)malloc(numCells_x * numCells_y * sizeof(Boid*));
    cellSizesHost = (uint*)malloc(numCells_x * numCells_y * sizeof(uint));
    cellOffsetsHost = (uint*)malloc(numCells_x * numCells_y * sizeof(uint));

    // Initialise array of boids and assign them to cells
    for(int i = 0; i < numBoids; i++) {
        float px = randFloat(0, xSize);
        float py = randFloat(0, ySize);
        // Random normalised direction
        float randTheta = randFloat(0, 2 * PI);
        float vx = cos(randTheta) * 1.0;
        float vy = sin(randTheta) * 1.0;
        boidsArrayHost[i] = Boid(px, py, vx, vy);
    }
}


/*
int main()
{
    int numBoids = 1000;
    int numFrames = 300;



    // Create a file and open it for writing
    FILE *fptr;
    fptr = fopen(filepath, "w");
    if (fptr == NULL) {
        printf("%s", "Error opening file");
        return 1;
    }



    //Variables for main
    setVars(512, 512, 64, 0.2, 8, 0.15, 20, 0.05, 0.2, 1, 2, 0.7 * PI);

    init(numBoids);

    save(fptr, numBoids, 0);



    // Update boids
    for (int frame = 1; frame < numFrames; frame++) {
        //Change the state and position of boids on the GPU
        updateBoids_GPU(numBoids);
        
        save(fptr, numBoids, frame);
    }

    freeMemory();
    
    // Close the file
    fclose(fptr);



    return 0;
}
*/