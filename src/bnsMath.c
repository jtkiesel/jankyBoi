#include "bnsMath.h"

double sgn(double x) {
	return (x < 0) ? -1 : 1;
}

double max(double x, double y) {
	return (x > y) ? x : y;
}
