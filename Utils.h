#ifndef UTILS_H
#define UTILS_H

#include "string.h"
#include <stdbool.h>
#include <stdio.h>



#define M_PI 3.1415926535897932384626433
#define LINK1_LENGTH 60
#define LINK2_LENGTH 60


void itos(int num, char *str);
void concat(char *dest, char *src);
bool IK(float x, float y, int* theta1, int* theta2);


#endif // !UTILS
