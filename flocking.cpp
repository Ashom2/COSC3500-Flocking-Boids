#include <iostream>
#include <math.h>

# define PI 3.141592653589793238462643383279502884L

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
    int xSize = 128;
    int ySize = 128;
    const int numParticles = 100;

    const char *filepath = "data.txt";



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

    // Motion test
    for(int i=0; i<numParticles; i++) {
        arr[i].pos.x += 5;
    }
    save(fptr, arr, numParticles, 1);

    
    // Close the file
    fclose(fptr);


    return 0;
}