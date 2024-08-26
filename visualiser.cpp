#include <iostream>

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
Main.
*/
int main() {
    const char *filepath = "data.txt";
    const size_t arrLength = 20;

    FILE *fptr;
    // Read vector array from file
    size_t elementSize = sizeof(Vector2D);
    Vector2D readArray[arrLength];
    // Open the file for reading
    fptr = fopen(filepath, "r");
    if (fptr == NULL) {
        printf("%s", "Error opening file");
        return 1;
    }

    // Read data from the file
    size_t result = fread(readArray, elementSize, arrLength, fptr);
    for (int i = 0; i < result; i++) {
        printf("Vector %d: (%.2f, %.2f)\n", i, readArray[i].x, readArray[i].y);
    }
    
    // Close the file
    fclose(fptr);
}