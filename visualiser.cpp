#include <iostream>
#include <SFML/Graphics.hpp>

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

    Vector2D readArray[arrLength];

    FILE *fptr;
    // Read vector array from file
    size_t elementSize = sizeof(Vector2D);
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


    sf::RenderWindow window(sf::VideoMode(800, 600), "My window");
    // sf::RenderWindow window(
    //     sf::VideoMode(640, 480),
    //     "Hello World");
    // sf::CircleShape shape(200);
 
    // while (window.isOpen()) 
    // {
    //     sf::Event event;
    //     while (
    //         window.pollEvent(event))
    //         if (event.type ==
    //         sf::Event::Closed)
    //             window.close();
 
    //     window.clear();
    //     window.draw(shape);
    //     window.display();
    // }
    return 0;
}