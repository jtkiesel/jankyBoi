#ifndef VECTOR_H_
#define VECTOR_H_

typedef struct Vector {
	double size;
	double angle;
} Vector;

Vector vectorCreate(double size, double angle);

#endif  // VECTOR_H_
