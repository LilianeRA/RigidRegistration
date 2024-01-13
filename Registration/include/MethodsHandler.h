#ifndef METHODSHANDLER_H
#define METHODSHANDLER_H


#include "RigidRegistration.h"

class MethodsHandler 
{
    public:

        MethodsHandler(const std::string &maindir, const std::string &inputdir, const std::string &outputdir, const std::string &testname, int threads);
        virtual ~MethodsHandler();

        void SetMode(const std::string& mode);
        void SetMethod(const std::string& method, const std::string& match, const std::string& estimation, const double ctsf_percentage);
        void SetGTfile(const std::string& gtfilepath);
        void SetPointClouds(const std::string& sourcemesh, const std::string& targetmesh, int downscalestep, int totalholes, double holeradius);

        void SetTensorParameters(const double alphacut_degrees, const double alphaellipse_degrees, const double sigmaN);

        void SaveParameters();

        void Run();

    private:
		RigidRegistration *registration = nullptr;
		MethodsData *data = nullptr;

        // Matcher *matcher = nullptr; // Euclidean distance, CTSF, Lie Groups
        // Estimator *estimator = nullptr;   // ICP, SWC, Sparse ICP

        
};

#endif // RIGIDREGISTRATION_H
