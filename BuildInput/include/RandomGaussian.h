#ifndef RANDOMGAUSSIAN_H
#define RANDOMGAUSSIAN_H

//////////////////////////////////////////////////////////////////////////////
// Generate random numbers with Gaussian distribution with 0 mean and 1.0 as
// standard-deviation N(0, 1).
//////////////////////////////////////////////////////////////////////////////

#include "Random.h"

class RandomGaussian
{
public:
    RandomGaussian();
    virtual ~RandomGaussian();

    // SERIES SPECIFICATION: seed1 must be different than seed2.
    void m_SetSeed(unsigned int seed1, unsigned int seed2);

    // RANDOM NUMBER GENERATION
    double m_Random(); // Output random double in N(0, 1)

private:
    Random a_ngen1;    // Independent random generators
    Random a_ngen2;
    bool a_has_value;      // Flags the existence of a second value.
    double a_second_value; // Keeps the second value
};

#endif // RANDOMGAUSSIAN_H
