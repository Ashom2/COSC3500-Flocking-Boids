#include <iostream>
#include <math.h>

const float PI = 3.141592653589793238462643383279502884;



const int xSize = 512;
const int ySize = 512;
int leftMargin = 64;
int rightMargin = xSize - 64;
int bottomMargin = 64;
int topMargin = ySize - 64;
const int numParticles = 1000;

const char *filepath = "data.txt";

// How hard the boid can turn to avoid walls
const float turnFactor = 0.2;
// The distance within which separation occurs
const float avoidRange = 8;
// The rate at which separation occurs
const float avoidFactor = 0.05;
// The distance within which alignment occurs
const float visualRange = 20;
// The rate at which alignment occurs
const float matchingFactor = 0.05;
// The rate at which cohesion occurs
const float cohesionFactor = 0.08;
// The minimum speed of the boids
const float minSpeed = 1;
// The maximum speed of the boids
const float maxSpeed = 2;
// The formation angle
const float formationAngle = 0.7 * PI;



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

// Sourced from https://stackoverflow.com/questions/686353/random-float-number-generation
float randFloat(float min, float max) {
    return min + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(max-min)));
}

/*
Saves 
*/
void save(FILE *fptr, Boid arr[], int numParticles, int frameNumber) {
    // Write vector array to file
    fprintf(fptr, "Frame %d\n", frameNumber);
    for(int i=0; i<numParticles; i++) {
        fprintf(fptr, "%f %f %f %f\n", arr[i].px, arr[i].py, arr[i].vx, arr[i].vy);
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
Gets the cosine of the angle between vectors
*/
float cosAngle(float x1, float y1, float x2, float y2) {
    float dot = x1 * x2 + y1 * y2;
    float mag1 = mag(x1, y1);
    float mag2 = mag(x2, y2);
    return dot / mag1 / mag2;
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
    if (cosAngle(formationDir_x, formationDir_y, diffVector_x, diffVector_y) < 0) {
        // Get at the vector orthogonal to the formationVector and move in that direction
        float m = mag(formationVector_x, formationVector_y);
        float val = dot(diffVector_x, diffVector_y, formationVector_x, formationVector_y) / m / m;
        orthogonalVector_x = val * formationVector_x - diffVector_x;
        orthogonalVector_y = val * formationVector_y - diffVector_y;
    }
}

/*
Updates a single boid
*/
void updateBoid (int index, Boid arr[]) {
    Boid& b = arr[index];

    // Update boid position
    b.px = b.px + b.vx;
    b.py = b.py + b.vy;

    float avoidVector_x = 0, avoidVector_y = 0;
    float formationDir_x = 0, formationDir_y = 0;
    float formationPos_x = 0, formationPos_y = 0;
    int neighboringBoids = 0;
    // Loop through every other boid
    for(int j=0; j<numParticles; j++) {
        if (index == j) continue; //Ignore itself
        
        Boid& o = arr[j];

        float dist = mag(b.px - o.px, b.py - o.py);
        if (dist < avoidRange) { // If the distance is less than protected range
            //Divide by the square of distance to make avoidance exponential and smoother
            avoidVector_x += (b.px - o.px) / (dist * dist);
            avoidVector_y += (b.py - o.py) / (dist * dist);
        }
        if (dist < visualRange) { // If the distance is less than visual range
            formationDir_x += o.vx;
            formationDir_y += o.vy;
            formationPos_x += o.px;
            formationPos_y += o.py;
            neighboringBoids++;
        }
    }

    // Separation - move away from nearby boids
    b.vx += avoidVector_x * avoidFactor;
    b.vy += avoidVector_y * avoidFactor;

    if (neighboringBoids > 0) { //If there were any boids in visual range
        // Get mean formation direction and position
        formationDir_x = formationDir_x / neighboringBoids;
        formationDir_y = formationDir_y / neighboringBoids;
        formationPos_x = formationPos_x / neighboringBoids;
        formationPos_y = formationPos_y / neighboringBoids;  

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
    float speed = sqrt(b.vx * b.vx + b.vy * b.vy);
    if (speed > maxSpeed) {
        b.vx = (b.vx / speed) * maxSpeed;
        b.vy = (b.vy / speed) * maxSpeed;
    }
    else if (speed == 0) {
        // TODO
    }
    else if (speed < minSpeed) {
        b.vx = (b.vx / speed) * minSpeed;
        b.vy = (b.vy / speed) * minSpeed;
    }
    //---------------------------------------
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



    // Initialise array of boids
    Boid arr[numParticles];
    for(int i=0; i<numParticles; i++) {
        float px = randFloat(0, xSize);
        float py = randFloat(0, ySize);
        // Random normalised direction
        float randTheta = randFloat(0, 2 * PI);
        float vx = cos(randTheta) * minSpeed;
        float vy = sin(randTheta) * minSpeed;
        arr[i] = Boid(px, py, vx, vy);
    }

    save(fptr, arr, numParticles, 0);



    // Update boids
    for (int frame=1; frame<300; frame++) {
        for(int i=0; i<numParticles; i++) {
            updateBoid(i, arr);
        }

        save(fptr, arr, numParticles, frame);
    }

    
    // Close the file
    fclose(fptr);


    return 0;
}