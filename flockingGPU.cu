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
//const int cellSize = pow(2, ceil(log2(std::max(avoidRange, visualRange))));
// I failed to implement this using constexpr so you (the user) must compute manually
// const int numCells_x = xSize / cellSize;
// const int numCells_y = ySize / cellSize;
// const int numCells_x = 16;
// const int numCells_y = 16;






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
Boid class to represent a single bird / particle / actor
*/
class Boid {
    public:
        float px;
        float py;
        float vx;
        float vy;

        Boid() {
            this->px = 0;
            this->py = 0;
            this->vx = 0;
            this->vy = 0;
        }
    
        Boid(float px, float py, float vx, float vy) {
            this->px = px;
            this->py = py;
            this->vx = vx;
            this->vy = vy;
        }
};

Boid* boidsArrayHost;



/*
Cell class to represent a subdivision of the simulation space containing a list of pointers to boids
*/
struct Cell {
    std::list<Boid*> boids;
};



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
__global__ void updateBoidsKernel_GPU(int N, Boid* in, Boid* out)
{
    //This boid's index
    int thisIndex = blockIdx.x * blockDim.x + threadIdx.x;

    if (thisIndex < N) { //Check out of bounds
        float avoidVector_x = 0, avoidVector_y = 0;
        float formationDir_x = 0, formationDir_y = 0;
        float formationPos_x = 0, formationPos_y = 0;
        int neighboringBoids = 0;

        //For some reason Boid& o = in[otherIndex]; seems to make it faster but making this line a reference makes it slower
        Boid b = in[thisIndex];

        for (int otherIndex = 0; otherIndex < N; otherIndex++) {
            if (otherIndex == thisIndex) continue; //Ignore itself

            Boid& o = in[otherIndex];
        
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



__host__ float updateBoids_GPU(int N)
{
    size_t size = N * sizeof(Boid);

    //Allocate memory on the device
    Boid *deviceIn;
    Boid *deviceOut;
    cudaMalloc(&deviceIn, size);
    cudaMalloc(&deviceOut, size);

    //Copy memory from host to device
    cudaMemcpy(deviceIn, boidsArrayHost, size, cudaMemcpyHostToDevice);

    //Specify blocks and threads, using 1D threads and blocks since our data is 1D
    int threadsPerBlock = BLOCKSIZE;
    int numBlocks = (N + threadsPerBlock - 1) / threadsPerBlock; //Int division of N / threadsPerBlock

    cudaEvent_t startGPU;
    cudaEvent_t stopGPU;
    cudaEventCreate(&startGPU);
    cudaEventCreate(&stopGPU);
    //Start Timer
    cudaEventRecord(startGPU);
    
    //Run
    updateBoidsKernel_GPU<<<numBlocks, threadsPerBlock>>>(N, deviceIn, deviceOut);

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
    cudaMemcpy(boidsArrayHost, deviceOut, size, cudaMemcpyDeviceToHost);

    //Clean up
    cudaFree(deviceIn);
    cudaFree(deviceOut);

    return kernelTime * 1000;
}



void init(int numBoids)
{
    boidsArrayHost = (Boid*)malloc(numBoids * sizeof(Boid));

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
        updateBoids_GPU(numBoids);
        
        save(fptr, numBoids, frame);
    }


    
    // Close the file
    fclose(fptr);



    return 0;
}
*/