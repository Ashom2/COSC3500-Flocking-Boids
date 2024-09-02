#include <iostream>
#include <math.h>

# define PI 3.141592653589793238462643383279502884L



int xSize = 512;
int ySize = 512;
const int numParticles = 1000;

const char *filepath = "data.txt";

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
const float minSpeed = 1;
// The maximum speed of the boids
const float maxSpeed = 3;




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

        float magnitude() {
            return sqrt(x * x + y * y);
        }

        float sqrMagnitude() {
            return x * x + y * y;
        }

        Vector2D operator+ (const Vector2D& other) const {
            return Vector2D(x + other.x, y + other.y);
        }

        Vector2D operator* (const float& other) const {
            return Vector2D(x * other, y * other);
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



    // Initialise array of boids
    Boid arr[numParticles];
    for(int i=0; i<numParticles; i++) {
        Vector2D pos = Vector2D(randFloat(0, xSize), randFloat(0, ySize));
        Vector2D dir = randDir() * minSpeed;
        arr[i] = Boid(pos, dir);
    }

    save(fptr, arr, numParticles, 0);

    // Update boids
    for (int frame=1; frame<100; frame++) {
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



            // 3. Cohesion - turn towards the center of mass of other boids
            // Loop through every other boid
            float avg_xpos = 0, avg_ypos = 0;
            neighboringBoids = 0;
            for(int j=0; j<numParticles; j++) {
                if (i == j) continue; //Ignore itself

                Boid& o = arr[j];

                // If the distance to other boid is less than visual range
                if (b.pos.distanceTo(o.pos) < visualRange) { 
                    avg_xpos += o.pos.x;
                    avg_ypos += o.pos.y;
                    neighboringBoids++;
                }
            }
            // Get the mean position of all boids in visual range
            if (neighboringBoids > 0) {
                avg_xpos = avg_xpos / neighboringBoids;
                avg_ypos = avg_ypos / neighboringBoids;
            }
            b.dir.x += (avg_xpos - b.pos.x) * cohesionFactor;
            b.dir.y += (avg_ypos - b.pos.y) * cohesionFactor;
            //---------------------------------------

            // Impose speed limit on boid
            float speed = b.dir.magnitude();
            if (speed > maxSpeed) {
                b.dir.x = (b.dir.x / speed) * maxSpeed;
                b.dir.y = (b.dir.y / speed) * maxSpeed;
            }
            else if (speed == 0) {
                // TODO
            }
            else if (speed < minSpeed) {
                b.dir.x = (b.dir.x / speed) * minSpeed;
                b.dir.y = (b.dir.y / speed) * minSpeed;
            }
            //---------------------------------------



            // Update boid position
            b.pos = b.pos + b.dir;
            // arr[i].pos = b.pos;
            // arr[i].dir = b.dir;
            // arr[i].pos = arr[i].pos + arr[i].dir;
        }
        save(fptr, arr, numParticles, frame);
    }

    
    // Close the file
    fclose(fptr);


    return 0;
}