#include <stdlib.h>
#include <stdio.h>

int main(void) {
    double A[5] = {
        [0] = 9.0,
        [1] = 2.1,
        [3] = 3.E+25,
        [4] = 0.0007,
    };

    for (size_t i =0; i < 5; ++i) {
        printf("element %zu is %g, \t its square is %g\n",
                i, A[i], A[i]*A[i]  );
    }

    return EXIT_SUCCESS;
}