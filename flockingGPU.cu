#include <stdio.h>
#include <stdarg.h>

const char *filepath = "TestOutput.txt";

int main()
{
    // Create a file and open it for writing
    FILE *fptr;
    fptr = fopen(filepath, "w");
    if (fptr == NULL) {
        printf("%s", "Error opening file");
        return 1;
    }

    fprintf(fptr, "Hello, world\n");
}