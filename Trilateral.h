#ifndef TRILATERAL_H
#define TRILATERAL_H
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

int double_equals(double a, double b);

double distance_sqr(struct point_t* a, struct point_t* b);

double distance(struct point_t* a, struct point_t* b);

int insect(struct circle_t circles[], struct point_t points[]);

void Cross_Point(struct circle_t circles[], struct point_t Location[]);

double norm(struct point p); // get the norm of a vector 求向量的范数

struct point trilateration_1(struct point point1, struct point point2, struct point point3, double r1, double r2, double r3);

void test();

struct point_t
{
    double x, y;
};

struct circle_t
{
    struct point_t center;
    double r;
};

struct point
{
    double x, y;
};
#endif

