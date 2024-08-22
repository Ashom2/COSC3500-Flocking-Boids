#include <iostream>

/*
Vector2D class for storing two-dimensional spacial vectors.
*/
class Vector2D { //TODO move to header
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
    //Vector2D arr[20];
    Vector2D v1 = Vector2D(1.00, 2);

    Vector2D arr[20];
    for(int i=0; i<20; i++) {
        arr[i] = Vector2D(-1, 5);
    }

    printf("%.2f", arr[0].x);
    //std::cout << "Flocking/swarming\n" << roundf(v1.x * 100) / 100;
    return 0;
}