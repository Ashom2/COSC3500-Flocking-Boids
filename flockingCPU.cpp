#include "flockingCPU.h"

const float PI = 3.141592653589793238462643383279502884;



const int xSize = 512;
const int ySize = 512;
int leftMargin = 64;
int rightMargin = xSize - 64;
int bottomMargin = 64;
int topMargin = ySize - 64;
const int numBoids = 1000;
const int numFrames = 300;

const char *filepath = "data.txt";

// How hard the boid can turn to avoid walls
const float turnFactor = 0.2;
// The distance within which separation occurs
const float avoidRange = 8;
// The rate at which separation occurs
const float avoidFactor = 0.15;
// The distance within which alignment occurs
const float visualRange = 20;
// The rate at which alignment occurs
const float matchingFactor = 0.05;
// The rate at which cohesion occurs
const float cohesionFactor = 0.2;
// The minimum speed of the boids
const float minSpeed = 1;
// The maximum speed of the boids
const float maxSpeed = 2;
// The formation angle
const float formationAngle = 0.7 * PI;

// Precalculated constants
const float sqrAvoidRange = avoidRange * avoidRange;
const float sqrVisualRange = visualRange * visualRange;
// Calculate cell size
const int cellSize = pow(2, ceil(log2(std::max(avoidRange, visualRange))));
// I failed to implement this using constexpr so you (the user) must compute manually
// const int numCells_x = xSize / cellSize;
// const int numCells_y = ySize / cellSize;
const int numCells_x = 16;
const int numCells_y = 16;



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

/*
Cell class to represent a subdivision of the simulation space containing a list of pointers to boids
*/
struct Cell {
    std::list<Boid*> boids;
};

// Sourced from https://stackoverflow.com/questions/686353/random-float-number-generation
float randFloat(float min, float max) {
    return min + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(max-min)));
}

/*
Saves 
*/
void save(FILE *fptr, Boid boidsArray[], int numBoids, int frameNumber) {
    // Write vector array to file
    fprintf(fptr, "Frame %d\n", frameNumber);
    for(int i=0; i<numBoids; i++) {
        fprintf(fptr, "%f %f %f %f\n", boidsArray[i].px, boidsArray[i].py, boidsArray[i].vx, boidsArray[i].vy);
    }    
}

/*
Magnitude of a vector
*/
float mag(float x, float y) {
    return sqrt(x * x + y * y);
}

/*
Square magnitude of a vector
*/
float sqrMag(float x, float y) {
    return x * x + y * y;
}

/*
Vector dot product
*/
float dot(float x1, float y1, float x2, float y2) {
    return x1 * x2 + y1 * y2;
}

/*
Gets the orthogonal vector using pass-by-reference
*/
void getOrthogonal(float &orthogonalVector_x, float &orthogonalVector_y, 
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

/*
Updates a single boid in a cell
*/
void updateBoidCell(Boid& b, Cell cellsArray[], int cell_x, int cell_y) {
    float avoidVector_x = 0, avoidVector_y = 0;
    float formationDir_x = 0, formationDir_y = 0;
    float formationPos_x = 0, formationPos_y = 0;
    int neighboringBoids = 0;    

    // // Iterate over neighboring cells
    for(int x = cell_x - 1; x <= cell_x + 1; x++) {
        if (x < 0 || x >= numCells_x) continue; //Ignore cells beyond boundary
        for(int y = cell_y - 1; y <= cell_y + 1; y++) {
            if (y < 0 || y >= numCells_y) continue; //Ignore cells beyond boundary

            // Iterate over boids in nieghboring cell
            for (auto const& i : cellsArray[x + y * numCells_x].boids) {
                Boid& o = *i;
                
                if (&b == &o) continue; //Ignore itself
        
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

    // Separation - move away from nearby boids
    b.vx += avoidVector_x * avoidFactor;
    b.vy += avoidVector_y * avoidFactor;

    if (neighboringBoids > 0) { //If there were any boids in visual range
        // Get mean formation direction and position
        formationDir_x /= neighboringBoids;
        formationDir_y /= neighboringBoids;
        formationPos_x /= neighboringBoids;
        formationPos_y /= neighboringBoids;  

        // Alignment - match the mean velocity of all boids in visual range
        b.vx += (formationDir_x - b.vx) * matchingFactor;
        b.vy += (formationDir_y - b.vy) * matchingFactor;

        // Flocking
        // Represents a vector pointed dowards this boid from the centre of mass
        float diffVector_x = b.px - formationPos_x;
        float diffVector_y = b.py - formationPos_y;

        float orthogonalVector_x = 0, orthogonalVector_y = 0;
        getOrthogonal(orthogonalVector_x, orthogonalVector_y, 
                diffVector_x, diffVector_y,
                formationDir_x, formationDir_y);

        b.vx += orthogonalVector_x * cohesionFactor;
        b.vy += orthogonalVector_y * cohesionFactor;            
    }



    // Avoid edges
    if (b.px < leftMargin) {
        b.vx += turnFactor;
    }
    else if (b.px > rightMargin) {
        b.vx -= turnFactor;
    }
    if (b.py < bottomMargin) {
        b.vy += turnFactor;
    }
    else if (b.py > topMargin) {
        b.vy -= turnFactor;
    }
    //---------------------------------------



    // Impose speed limit on boid
    //TODO use *= and precalculate maxSpeed / speed
    float speed = mag(b.vx, b.vy);
    if (speed > maxSpeed) {
        b.vx *= maxSpeed / speed;
        b.vy *= maxSpeed / speed;
    }
    else if (speed == 0) {
        // TODO
    }
    else if (speed < minSpeed) {
        b.vx *= minSpeed / speed;
        b.vy *= minSpeed / speed;
    }
    //---------------------------------------

    
    
    // Update boid position
    b.px += b.vx;
    b.py += b.vy;
    //---------------------------------------
}

/*
Retrieves the x index of a cell at position x
*/
int getCell_x(float x) {
    //TODO check out std::clamp
    int nx = (int)(x / cellSize);
    if (nx > numCells_x - 1) return numCells_x - 1;
    if (nx < 0) return 0;
    return nx;
}

/*
Retrieves the y index of a cell at position y
*/
int getCell_y(float y) {
    //TODO check out std::clamp
    int ny = (int)(y / cellSize);
    if (ny > numCells_y - 1) return numCells_y - 1;
    if (ny < 0) return 0;
    return ny;
}

/*
Retrieves the 1D index of a cell at position x and y
*/
int getCell_i(float x, float y) {
    //TODO check out std::clamp
    return getCell_x(x) + getCell_y(y) * numCells_x;
}



void updateCell(Cell cellsArray[], int cx, int cy) {
    //TODO I was working on an optimisation to save all neighboring cell's boids to an array to be reused for the whole cell
    // printf("updating cell\n");
    // Construct array of all boids in neighboring cells
    // Boid** neighboringBoids = (Boid**)malloc(1 * sizeof(Boid*));
    // int index = 0;
    // // Iterate over neighboring cells
    // for(int x = cx - 1; x <= cx + 1; x++) {
    //     if (x < 0 || x >= numCells_x) continue; //Ignore cells beyond boundary
    //     for(int y = cy - 1; y <= cy + 1; y++) {
    //         if (y < 0 || y >= numCells_y) continue; //Ignore cells beyond boundary

    //         neighboringBoids = (Boid**)realloc(neighboringBoids, (1 + index + cellsArray[x][y].boids.size()) * sizeof(Boid*));
    //         // Iterate over boids in nieghboring cell
    //         for (auto const& i : cellsArray[x][y].boids) {
    //             Boid& o = *i;
    //             neighboringBoids[index++] = &o;
    //         }
    //     }
    // }
    // printf("%d\n", index);

    // For each boid in the cell
    int ci = cx + cy * numCells_x;
    for (auto it = cellsArray[ci].boids.begin(); it!=cellsArray[ci].boids.end(); it++) {
        Boid& b = **it; 

        updateBoidCell(b, cellsArray, cx, cy);

        // Update boid in cell
        int ni = getCell_i(b.px, b.py);
        if (ni != ci) {
            cellsArray[ni].boids.push_back(&b);
            it = cellsArray[ci].boids.erase(it);
        }
    }
}

void updateFrame(Cell cellsArray[])
{
    // TODO could potentially be faster if we fed a lookup table to the functions
    for(int x=0; x<numCells_x; x++) {
        for(int y=0; y<numCells_y; y++) {
            updateCell(cellsArray, x, y);
        }
    }
}

Boid* initBoids(int numBoids)
{
    // Initialise array of boids
    Boid* boidsArray = (Boid*)malloc(numBoids * sizeof(Boid));

    //Give each a random position
    for(int i = 0; i < numBoids; i++) {
        float px = randFloat(0, xSize);
        float py = randFloat(0, ySize);
        // Random normalised direction
        float randTheta = randFloat(0, 2 * PI);
        float vx = cos(randTheta) * minSpeed;
        float vy = sin(randTheta) * minSpeed;
        boidsArray[i] = Boid(px, py, vx, vy);
    }

    return boidsArray;
}

Cell* initCells(Boid* boidsArray, Cell* cellsArray)
{
    for(int i = 0; i < numBoids; i++) {
        //Add pointer to boid to cell
        cellsArray[getCell_i(boidsArray[i].px, boidsArray[i].py)].boids.push_back(&boidsArray[i]);
    }
    return cellsArray;
}

/*
Main.
*/
int main() {
    // Create a file and open it for writing
    FILE *fptr;
    fptr = fopen(filepath, "w");
    if (fptr == NULL) {
        printf("%s", "Error opening file");
        return 1;
    }
    
    // Initialise arrays
    Boid* boidsArray = initBoids(numBoids);
    Cell cellsArray[numCells_x * numCells_y];
    initCells(boidsArray, cellsArray);    

    save(fptr, boidsArray, numBoids, 0);

    // Update boids
    for (int frame=1; frame<numFrames; frame++) {
        // For each cell update each boid inside
        updateFrame(cellsArray);
        
        save(fptr, boidsArray, numBoids, frame);
    }

    // Close the file
    fclose(fptr);

    return 0;
}