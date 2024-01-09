#ifndef RIGIDREGISTRATION_H
#define RIGIDREGISTRATION_H


#include "MethodsData.h"

class RigidRegistration 
{
    public:

        RigidRegistration(const std::string &maindir, const std::string &inputdir, const std::string &outputdir, const std::string &testname, int threads);
        virtual ~RigidRegistration();

        void SetMode(const std::string& mode);
        void SetMethod(const std::string& method, const std::string& match, const std::string& estimation);
        void SetGTfile(const std::string& gtfilepath);
        void SetPointClouds(const std::string& sourcemesh, const std::string& targetmesh, int downscalestep, int totalholes, double holeradius);

        void SetTensorParameters(const double alphacut_degrees, const double alphaellipse_degrees, const double sigmaN);

        void SaveParameters();

        void Run();

    protected:

        double currIterationWeight = 1e5;
        double maxIterationWeight = 1e5;
        double minIterationWeight = 1e-6;
        double stepIterationWeight =  0.1;

        int iterationCounter = 0;

        const double initialError = 1e10;
        double trimming = 1.0;
        int correspondences = 0;

        void Iteration();
        // measures the distance of the points between two point clouds
        void MatchPointClouds();  // Euclidean, CTSF, Lie Group
        void EstimateTransformation(); // ICP, GMM, Super 4PCS, 

    private:
		MethodsData *data = nullptr;

        // Matcher *matcher = nullptr; // Euclidean distance, CTSF, Lie Groups
        // Estimator *estimator = nullptr;   // ICP, SWC, Sparse ICP

        
};

#endif // RIGIDREGISTRATION_H
