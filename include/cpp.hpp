#ifndef CPP_HPP_
#define CPP_HPP_

#include <cstddef>

void *operator new(unsigned int n);

void operator delete(void *p);

void operator delete(void *p, unsigned int n);

namespace std {

void __throw_bad_alloc();

void __throw_out_of_range_fmt(char const* fmt, ...);

}  // namespace std

#endif  // CPP_HPP_
