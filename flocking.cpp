#include <iostream>
#include <math.h>

# define PI 3.141592653589793238462643383279502884L



int xSize = 128;
int ySize = 128;
const int numParticles = 100;

const char *filepath = "data.txt";

// The distance within which separation occurs
const float protectedRange = 8;
// The rate at which separation occurs
const float avoidFactor = 0.05;
// The distance within which alignment occurs
const float visualRange = 40;
// The rate at which alignment occurs
const float matchingFactor = 0.05;



/*
Vector2D class for storing two-dimensional spacial vectors.
*/
class Vector2D {
    public:
        float x;
        float y;

        Vector2D() {
            this->x = 0;
            this->y = 0;
        }

        Vector2D(float x, float y) {
            this->x = x;
            this->y = y;
        }

        float distanceTo(Vector2D other) {
            float dx = x - other.x;
            float dy = y - other.y;
            return sqrt(dx * dy + dx * dy);
        }

        Vector2D operator+ (const Vector2D & other) const {
            return Vector2D(x + other.x, y + other.y);
        }
};

/*
Boid class to represent a single bird / particle / actor
*/
class Boid {
    public:
        Vector2D pos;
        Vector2D dir;

        Boid() {
            this->pos = Vector2D();
            this->dir = Vector2D();
        }
    
        Boid(Vector2D pos, Vector2D dir) {
            this->pos = pos;
            this->dir = dir;
        }
};

// Sourced from https://stackoverflow.com/questions/686353/random-float-number-generation
float randFloat(float min, float max) {
    return min + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(max-min)));
}

/*
Gets a random direction represented by a normalised vector
*/
Vector2D randDir() {
    float theta = randFloat(0, 2 * PI);
    return Vector2D(cos(theta), sin(theta));
}

/*
Saves 
*/
void save(FILE *fptr, Boid arr[], int numParticles, int frameNumber) {
    // Write vector array to file
    fprintf(fptr, "Frame %d\n", frameNumber);
    for(int i=0; i<numParticles; i++) {
        fprintf(fptr, "%f %f %f %f\n", arr[i].pos.x, arr[i].pos.y, arr[i].dir.x, arr[i].dir.y);
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



    // Initialise vector array
    Boid arr[numParticles];
    for(int i=0; i<numParticles; i++) {
        Vector2D pos = Vector2D(randFloat(0, xSize), randFloat(0, ySize));
        Vector2D dir = randDir();
        arr[i] = Boid(pos, dir);
    }

    save(fptr, arr, numParticles, 0);

    // Update boids
    for (int frame=1; frame<50; frame++) {
        for(int i=0; i<numParticles; i++) {
            Boid& b = arr[i];

            // 1. Separation - attempt to avoid other close boids
            float close_dx = 0, close_dy = 0;
            // Loop through every other boid
            for(int j=0; j<numParticles; j++) {
                if (i == j) continue; //Ignore itself

                Boid& o = arr[j];

                // If the distance is less than protected range
                if (b.pos.distanceTo(o.pos) < protectedRange) { 
                    close_dx += b.pos.x - o.pos.x;
                    close_dy += b.pos.y - o.pos.y;
                }
            }
            b.dir.x += close_dx * avoidFactor;
            b.dir.y += close_dy * avoidFactor;
            //---------------------------------------

            // 2. Alignment - attempt to match velocity of nearby boids
            float avg_xvel = 0, avg_yvel = 0;
            int neighboringBoids = 0;
            // Loop through every other boid
            for(int j=0; j<numParticles; j++) {
                if (i == j) continue; //Ignore itself

                Boid& o = arr[j];

                // If the distance to other boid is less than visual range
                if (b.pos.distanceTo(o.pos) < visualRange) { 
                    avg_xvel += o.dir.x;
                    avg_yvel += o.dir.y;
                    neighboringBoids++;
                }
            }
            // Get the mean velocity of all boids in visual range
            if (neighboringBoids > 0) {
                avg_xvel = avg_xvel / neighboringBoids;
                avg_yvel = avg_yvel / neighboringBoids;
            }
            b.dir.x += (avg_xvel - b.dir.x) * matchingFactor;
            b.dir.y += (avg_yvel - b.dir.y) * matchingFactor;
            //---------------------------------------


            b.pos = b.pos + b.dir;
        }
        save(fptr, arr, numParticles, frame);
    }

    
    // Close the file
    fclose(fptr);


    return 0;
}