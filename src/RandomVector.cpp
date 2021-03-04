/*************************************************************************************
    GCG - GROUP FOR COMPUTER GRAPHICS
        Universidade Federal de Juiz de Fora
        Instituto de Ciências Exatas
        Departamento de Ciência da Computação

   RANDOM.CPP: functions for random number generation.

   Marcelo Bernardes Vieira
   Eder de Almeida Perez
**************************************************************************************/

#include "RandomVector.h"
#include <random>

RandomVector::RandomVector()
{
    //ctor
    m_SetSeed(rand(), rand());
}

RandomVector::~RandomVector()
{
    //dtor
}

// SERIES SPECIFICATION: seed1 must be different than seed2.
void RandomVector::m_SetSeed(unsigned int seed1, unsigned int seed2)
{
    a_ngen1.m_SetSeed(seed1);
    a_ngen2.m_SetSeed(seed2);
}

// RANDOM VECTOR GENERATION
void RandomVector::m_FloatRandomVec(float &x, float &y, float &z)
{
    float fi = (float) ((2.0 * M_PI) * a_ngen1.m_FloatRandom()); // Tira angulo aleatorio

    z = (float)  (1.0 - 2.0 * a_ngen2.m_FloatRandom()); // Calcula componente Z
    x = (float)  (std::sqrt(1.0 - z*z) * std::cos(fi)); // Calcula componente X
    y = (float)  (x * std::tan(fi));  // Calcula componente Y
}

void RandomVector::m_DoubleRandomVec(double &x, double &y, double &z)
{
    double fi = (double) (2.0 * M_PI) * a_ngen1.m_DoubleRandom(); // Tira angulo aleatorio

    z = (double)  (1.0 - 2.0 * a_ngen2.m_DoubleRandom()); // Calcula componente Z
    x = (double)  (std::sqrt(1.0 - z*z) * std::cos(fi)); // Calcula componente X
    y = (double)  (x * std::tan(fi));  // Calcula componente Y
}







