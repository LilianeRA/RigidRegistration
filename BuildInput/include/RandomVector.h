#ifndef RANDOMVECTOR_H
#define RANDOMVECTOR_H

////////////////////////////////////////////////////////////////
// Class to generate vectors with isotropic distribution.
////////////////////////////////////////////////////////////////
#include "Random.h"

class RandomVector
{
public:
    RandomVector();
    virtual ~RandomVector();
    // SERIES SPECIFICATION: seed1 must be different than seed2.
    void m_SetSeed(unsigned int seed1, unsigned int seed2);

    // RANDOM VECTOR GENERATION
    void m_FloatRandomVec(float &x, float &y, float &z);    // Output isotropic float vector in S^2.
    void m_DoubleRandomVec(double &x, double &y, double &z); // Output isotropic double vector in S^2.

private:
    Random a_ngen1;  // Independent random generators
    Random a_ngen2;
};

#endif // RANDOMVECTOR_H
