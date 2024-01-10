#ifndef RIGIDREGISTRATION_H
#define RIGIDREGISTRATION_H


#include "MethodsData.h"

class RigidRegistration 
{
    public:

        RigidRegistration(const MethodsData *methodsData);
        virtual ~RigidRegistration();

        void Run();

    private:
		const MethodsData* data; // the first time this variable is seted, its done

        double currIterationWeight = 1e5;
        double maxIterationWeight = 1e5;
        double minIterationWeight = 1e-6;
        const double stepIterationWeight = 0.1;

        int iterationCounter = 0;

        const double initialError = 1e10;
        double trimming = 1.0;
        int correspondences = 0; 

        std::vector<int> tgt2src_correspondence;
        // Matcher *matcher = nullptr; // Euclidean distance, CTSF, Lie Groups
        // Estimator *estimator = nullptr;   // ICP, SWC, Sparse ICP

        std::function< double(const Point*, const Point*, const double) > distanceFunction;

        void Setup();
        void MatchPointClouds();
};

#endif // RIGIDREGISTRATION_H
