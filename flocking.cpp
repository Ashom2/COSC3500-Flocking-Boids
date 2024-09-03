#include <iostream>
#include <math.h>

const float PI = 3.141592653589793238462643383279502884;



const int xSize = 512;
const int ySize = 512;
int leftMargin = 64;
int rightMargin = xSize - 64;
int bottomMargin = 64;
int topMargin = ySize - 64;
const int numParticles = 300;

const char *filepath = "data.txt";

// How hard the boid can turn to avoid walls
const float turnFactor = 0.2;
// The distance within which separation occurs
const float protectedRange = 8;
// The rate at which separation occurs
const float avoidFactor = 0.05;
// The distance within which alignment occurs
const float visualRange = 40;
// The rate at which alignment occurs
const float matchingFactor = 0.05;
// The rate at which cohesion occurs
const float cohesionFactor = 0.0005;
// The minimum speed of the boids
const float minSpeed = 3;
// The maximum speed of the boids
const float maxSpeed = 6;
// The formation angle
const float formationAngle = PI;



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
    float mag1 = sqrt(x1 * x1 + y1 * y1);
    float mag2 = sqrt(x2 * x2 + y2 * y2);
    return dot / mag1 / mag2;
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
            Boid& b = arr[i];

            float avoidVector_x = 0, avoidVector_y = 0;
            float formationDir_x = 0, formationDir_y = 0;
            float formationPos_x = 0, formationPos_y = 0;
            int neighboringBoids = 0;
            // Loop through every other boid
            for(int j=0; j<numParticles; j++) {
                if (i == j) continue; //Ignore itself
                
                Boid& o = arr[j];

                float dx = b.px - o.px;
                float dy = b.py - o.py;
                float dist = sqrt(dx * dx + dy * dy);
                if (dist < protectedRange) { // If the distance is less than protected range
                    avoidVector_x += b.px - o.px;
                    avoidVector_y += b.py - o.py;
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

            if (neighboringBoids > 0) {
                formationDir_x = formationDir_x / neighboringBoids;
                formationDir_y = formationDir_y / neighboringBoids;
                formationPos_x = formationPos_x / neighboringBoids;
                formationPos_y = formationPos_y / neighboringBoids;  
            }
            else {
                formationDir_x = b.vx;
                formationDir_y = b.vy;
                formationPos_x = b.px;
                formationPos_y = b.py; 
            }
            // Alignment - match the mean velocity of all boids in visual range
            b.vx += (formationDir_x - b.vx) * matchingFactor;
            b.vy += (formationDir_y - b.vy) * matchingFactor;

            // Cohesion - get the mean position of all boids in visual range
            b.vx += (formationPos_x - b.px) * cohesionFactor;
            b.vy += (formationPos_y - b.py) * cohesionFactor;




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



            // Update boid position
            b.px = b.px + b.vx;
            b.py = b.py + b.vy;
        }

        save(fptr, arr, numParticles, frame);
    }

    
    // Close the file
    fclose(fptr);


    return 0;
}