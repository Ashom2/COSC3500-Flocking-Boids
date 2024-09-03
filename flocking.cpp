#include <iostream>
#include <math.h>

# define PI 3.141592653589793238462643383279502884L



const int xSize = 512;
const int ySize = 512;
int leftMargin = 64;
int rightMargin = xSize - 64;
int bottomMargin = 64;
int topMargin = ySize - 64;
const int numParticles = 400;

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

            float close_dx = 0, close_dy = 0;
            float avg_xvel = 0, avg_yvel = 0;
            float avg_xpos = 0, avg_ypos = 0;
            int neighboringBoids = 0;
            // Loop through every other boid
            for(int j=0; j<numParticles; j++) {
                if (i == j) continue; //Ignore itself
                
                Boid& o = arr[j];

                float dx = b.px - o.px;
                float dy = b.py - o.py;
                float dist = sqrt(dx * dx + dy * dy);
                if (dist < protectedRange) { // If the distance is less than protected range
                    close_dx += b.px - o.px;
                    close_dy += b.py - o.py;
                }
                if (dist < visualRange) { // If the distance is less than visual range
                    avg_xvel += o.vx;
                    avg_yvel += o.vy;
                    avg_xpos += o.px;
                    avg_ypos += o.py;
                    neighboringBoids++;
                }
            }

            // Separation - move away from nearby boids
            b.vx += close_dx * avoidFactor;
            b.vy += close_dy * avoidFactor;

            if (neighboringBoids > 0) {
                avg_xvel = avg_xvel / neighboringBoids;
                avg_yvel = avg_yvel / neighboringBoids;
                avg_xpos = avg_xpos / neighboringBoids;
                avg_ypos = avg_ypos / neighboringBoids;
            }
            // Alignment - match the mean velocity of all boids in visual range
            b.vx += (avg_xvel - b.vx) * matchingFactor;
            b.vy += (avg_yvel - b.vy) * matchingFactor;

            // Cohesion - get the mean position of all boids in visual range
            b.vx += (avg_xpos - b.px) * cohesionFactor;
            b.vy += (avg_ypos - b.py) * cohesionFactor;




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