#include "utility.h"

double getTimeStamp()
{
    struct timeval t;
    gettimeofday(&t, NULL);
    return (t.tv_sec * 1000000 + t.tv_usec) * 1e-6;
}