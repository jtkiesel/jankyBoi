#include "cpp.hpp"

#include "api.hpp"

#include <cstdlib>

void *operator new(unsigned int n) {
	return std::malloc(n);
}

void operator delete(void *p) {
	std::free(p);
}

void operator delete(void *p, unsigned int n) {
	std::free(p);
}

namespace std {

void __throw_bad_alloc() {
	pros::printf("Unable to allocate memory.\n");
}

void __throw_out_of_range_fmt(char const* fmt, ...) {
	pros::printf("Out of range: %s\n", fmt);
}

}  // namespace std
