#include <stdio.h>
#include <iostream>

#include "shared.h"

void TestSharedTypes(void)
{
    Vec2 a = {3.0, 4.0};
    Vec2 product = operator*(a,2.0);
    Vec2 division = operator/(a,2.0);

    printf("Length = %f\n", Length(a));
    printf("Product: (%f, %f)\n", product.x, product.y);
    printf("Division: (%f, %f)\n", division.x, division.y);
    printf("Difficulty = %d\n", HARD);
    printf("PI = %.10f\n", PI);
}

int main(void) {
    TestSharedTypes();
    return 0;
}