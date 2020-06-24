#include "utils/timelib.hpp"

void add_timespec(const timespec* t1, const timespec* d, timespec* out) {
	out->tv_nsec = d->tv_nsec + t1->tv_nsec;
	out->tv_sec = d->tv_sec + t1->tv_sec;

	if (out->tv_nsec > 1e9) {
		out->tv_sec += 1;
		out->tv_nsec -= 1e9;
	}
}

double sub_timespec(const timespec* t1, const timespec* t2) {
	double out;
	double sec = t1->tv_sec - t2->tv_sec;
	double nsec = t1->tv_nsec - t2->tv_nsec;	
	if (nsec < 0) {
		sec -= 1.0;
		nsec += 1e9;
	}

	out = sec + nsec / 1e9;
}
