#include "cpp.hpp"

#include <cstdlib>

void *operator new(unsigned int n) {
	return std::malloc(n);
}

void operator delete(void *p) {
	std::free(p);
}
