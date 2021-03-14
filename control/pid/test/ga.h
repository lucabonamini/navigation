#ifndef _GA_H_
#define _GA_H_

#include "pid/pid.h"

struct Chromosome {
    double Kp;
    double Ki;
    double Kd;
    double fitness;
};

struct Population {
    std::vector<Chromosome> chromosomes;
};

#endif