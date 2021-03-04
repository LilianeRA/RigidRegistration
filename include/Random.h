#ifndef RANDOM_H
#define RANDOM_H

//////////////////////////////////////////////////////////////////////////////
// Generate random numbers uniformly distributed with period (2^19937) - 1
//
//  M. Matsumoto & T. Nishimura, "Mersenne Twister: A 623-Dimensionally
//   Equidistributed Uniform Pseudo-Random Number Generator", ACM Transactions
//   on Modeling and Computer Simulation, vol. 8, no. 1, 1998, pp. 3-30.
//////////////////////////////////////////////////////////////////////////////
#include <limits.h>

class Random
{
public:
    Random();
    virtual ~Random();

    // series specification
    void m_SetSeed(unsigned int seed);
    // getting a random number
    // Output random integer in U[min, max]
    int m_IntRandom(int min_val=0, int max_val=INT_MAX);
    // Output random float in U[min, max]
    float m_FloatRandom(float min_val=0.0f, float max_val=1.0f);
    // Output random double in U[min, max]
    double m_DoubleRandom(double min_val=0.0, double max_val=1.0);

private:
    unsigned int m_BitRandom32(); // Generate 32 random bits

#define MERSENNE_N          624

    int a_mti;                          // Index for Mersenne Twisted state vector
    unsigned int a_last_interval;       // Last interval length for intRandom
    unsigned int a_R_limit;             // Rejection limit used by intRandom
    unsigned long a_mag01[2];           // mag01[x] = x * MATRIX_A for x=0,1
    unsigned long a_mt[MERSENNE_N];     // The array for the state vector
};

#endif // RANDOM_H
