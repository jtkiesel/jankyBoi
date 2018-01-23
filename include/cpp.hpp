#ifndef CPP_HPP_
#define CPP_HPP_

#include <cstddef>

void *operator new(unsigned int n);

//void operator delete(void *p);

void operator delete(void *p, unsigned int n);

#endif  // CPP_HPP_
