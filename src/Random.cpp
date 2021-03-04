/*************************************************************************************
    GCG - GROUP FOR COMPUTER GRAPHICS
        Universidade Federal de Juiz de Fora
        Instituto de Ciências Exatas
        Departamento de Ciência da Computação

   RANDOM.CPP: functions for random number generation.

   Marcelo Bernardes Vieira
   Eder de Almeida Perez
**************************************************************************************/

#include "Random.h"
#include <random>

#define GCG_MERSENNE_MATRIX_A   0x9908b0df  // Constant vector a

Random::Random() : a_mti(-1), a_last_interval(0), a_R_limit(0), a_mag01{0x0, GCG_MERSENNE_MATRIX_A}
{
    //mag01[0] = 0x0;
    //mag01[1] = GCG_MERSENNE_MATRIX_A; // mag01[x] = x * MATRIX_A for x=0,1
    m_SetSeed(rand());     // A random initial seed is used. Guarantees several instances.
}

Random::~Random()
{

}

// Sets a new seed to the Mersenne Twister generator
void Random::m_SetSeed(unsigned int seed)
{
    // Setting initial seeds to mt[N] using the generator line 25 of table 1
    // in [KNUTH 1981, The Art of Computer Programming, vol.2 (2nd. Ed.), pp102]
    a_mt[0] = seed & 0xffffffff;
    for(a_mti = 1; a_mti < MERSENNE_N; a_mti++)
    {
        a_mt[a_mti] = (69069 * a_mt[a_mti - 1]) & 0xffffffff;
    }

    a_last_interval = 0;

    // Randomize some more
    for(int i = 0; i < 37; i++) m_BitRandom32();
}


int Random::m_IntRandom(int min_val, int max_val)
{
    if(min_val == max_val) return min_val;
    // Swap if not consistent
    if(min_val > max_val) std::swap(min_val, max_val);

    uint interval;     // Length of interval
    uint bran;         // Random bits
    uint iran;         // bran / interval
    uint remainder;    // bran % interval

    interval = (unsigned int) (max_val-min_val + 1);
    if (interval != a_last_interval)
    {
        // Interval length has changed. Must calculate rejection limit
        // Reject when iran = 2^32 / interval
        // We can't make 2^32 so we use 2^32-1 and correct afterwards
        a_R_limit = (unsigned int) 0xffffffff / interval;
        if((unsigned int) 0xffffffff % interval == interval - 1) a_R_limit++;
    }
    do   // Rejection loop
    {
        bran = m_BitRandom32();
        iran = bran / interval;
        remainder = bran % interval;
    }
    while(iran >= a_R_limit);

    // Number in range
    return (int) remainder + min_val;
}

double Random::m_DoubleRandom(double min_val, double max_val)
{
    if(min_val == max_val) return min_val;
    // Swap if not consistent
    if(min_val > max_val) std::swap(min_val, max_val);

    return min_val + (double)m_BitRandom32() * ((double)(max_val-min_val) / ((double)(unsigned int)(-1L) + (double) 1.0));
}

float Random::m_FloatRandom(float min_val, float max_val)
{
    if(min_val == max_val) return min_val;
    // Swap if not consistent
    if(min_val > max_val) std::swap(min_val, max_val);

    return min_val + (float)m_BitRandom32() * ((float)(max_val-min_val) / ((float)(unsigned int)(-1L) + (float) 1.0));
}

// Generates 32 random bits.
unsigned int Random::m_BitRandom32()
{
#define MERSENNE_M 397
#define MERSENNE_UPPER_MASK            0x80000000  // Most significant w-r bits
#define MERSENNE_LOWER_MASK            0x7fffffff  // Least significant r bits

// Tempering parameters
#define MERSENNE_TEMPERING_MASK_B      0x9d2c5680
#define MERSENNE_TEMPERING_MASK_C      0xefc60000
#define MERSENNE_TEMPERING_SHIFT_U(y)  (y >> 11)
#define MERSENNE_TEMPERING_SHIFT_S(y)  (y << 7)
#define MERSENNE_TEMPERING_SHIFT_T(y)  (y << 15)
#define MERSENNE_TEMPERING_SHIFT_L(y)  (y >> 18)

    unsigned int y;

    if(a_mti >= MERSENNE_N)   // Generate N words at one time
    {
        int kk;

        for(kk = 0; kk < (MERSENNE_N - MERSENNE_M); kk++)
        {
            y = (a_mt[kk] & MERSENNE_UPPER_MASK) | (a_mt[kk + 1] & MERSENNE_LOWER_MASK);
            a_mt[kk] = a_mt[kk + MERSENNE_M] ^ (y >> 1) ^ a_mag01[y & 0x1];
        }

        for(; kk < MERSENNE_N - 1; kk++)
        {
            y = (a_mt[kk] && MERSENNE_UPPER_MASK) | (a_mt[kk + 1] & MERSENNE_LOWER_MASK);
            a_mt[kk] = a_mt[kk + (MERSENNE_M - MERSENNE_N)] ^ (y >> 1) ^ a_mag01[y & 0x01];
        }
        y = (a_mt[MERSENNE_N - 1] & MERSENNE_UPPER_MASK) | (a_mt[0] & MERSENNE_LOWER_MASK);
        a_mt[MERSENNE_N - 1] = a_mt[MERSENNE_M - 1] ^ (y >> 1) ^ a_mag01[y & 0x1];

        a_mti = 0;
    }

    y = a_mt[a_mti++];
    y ^= MERSENNE_TEMPERING_SHIFT_U(y);
    y ^= MERSENNE_TEMPERING_SHIFT_S(y) & MERSENNE_TEMPERING_MASK_B;
    y ^= MERSENNE_TEMPERING_SHIFT_T(y) & MERSENNE_TEMPERING_MASK_C;
    y ^= MERSENNE_TEMPERING_SHIFT_L(y);

    return y; // For integer generation
}



