#ifndef _TIMELIB_HPP_
#define _TIMELIB_HPP_

#include <time.h>

void add_timespec(const timespec* t1, const timespec* d, timespec* out);
double sub_timespec(const timespec* t1, const timespec* t2);
long unsigned int timespec2micro(const timespec* t);

#endif
