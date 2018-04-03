#include "Vector.h"

Vector vectorCreate(double size, double angle) {
	return (Vector) {.size = size, .angle = angle};
}
