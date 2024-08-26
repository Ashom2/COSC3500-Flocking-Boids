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

    // Initialise vector array
    Vector2D arr[20];
    for(int i=0; i<20; i++) {
        arr[i] = Vector2D(i, 20 - i);
    }



    // Write vector array to file
    // TODO use fstream (it was having problems before)
    FILE *fptr;
    // Create a file and open it for writing
    fptr = fopen(filepath, "w");
    if (fptr == NULL) {
        printf("%s", "Error opening file");
        return 1;
    }
    // Write some text to the file
    fwrite(arr, sizeof(Vector2D), sizeof(arr) / sizeof(arr[0]), fptr);
    // Close the file
    fclose(fptr);



    // Read vector array from file
    size_t elementSize = sizeof(Vector2D);
    const size_t arrLength = 20;
    Vector2D readArray[arrLength];
    // Open the file for reading
    fptr = fopen("data.txt", "r");
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

    

    return 0;
}