#include <iostream>

/*
Vector2D class for storing two-dimensional spacial vectors.
*/
class Vector2D { //TODO move to header
    public:
    float x;
    float y;

    Vector2D(float x, float y) {
        x = x;
        y = y;
    }
};

/*
Main.
*/
int main() {
    std::cout << "Flocking/swarming\n";
    return 0;
}