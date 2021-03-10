/*************************************************************************************
    GCG - GROUP FOR COMPUTER GRAPHICS
        Universidade Federal de Juiz de Fora
        Instituto de Ciências Exatas
        Departamento de Ciência da Computação

   RANDOM.CPP: functions for random number generation.

   Marcelo Bernardes Vieira
   Eder de Almeida Perez
**************************************************************************************/

#include "RandomGaussian.h"
#include <random>


RandomGaussian::RandomGaussian() : a_has_value(false), a_second_value(-1.0)
{
    //ctor
    // A random initial seed is used. Guarantees several instances.
    // C rand() funtion.
    m_SetSeed(rand(), rand());
}

RandomGaussian::~RandomGaussian()
{
    //dtor
}

// SERIES SPECIFICATION: seed1 must be different than seed2.
void RandomGaussian::m_SetSeed(unsigned int seed1, unsigned int seed2)
{
    a_ngen1.m_SetSeed(seed1);
    a_ngen2.m_SetSeed(seed2);
    a_has_value = false;
}

// RANDOM NUMBER GENERATION
// Output random double in N(0, 1)
double RandomGaussian::m_Random()
{
    // Checks the existence of the second value.
    if(a_has_value)
    {
        a_has_value = false;
        return a_second_value;
    }

    // Compute next two values
    a_has_value = true;
    double u1 = (double) std::sqrt(-2.0 * std::log(a_ngen1.m_DoubleRandom()));
    double u2 = (double) (2.0 * M_PI * a_ngen2.m_DoubleRandom());
    a_second_value = u1 * std::sin(u2); // keeps the second value

    return u1 * std::cos(u2); // returns the first one
}


