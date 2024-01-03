#ifndef RIGIDREGISTRATION_H
#define RIGIDREGISTRATION_H


#include "MethodsData.h"

class RigidRegistration 
{
    public:

        RigidRegistration(const std::string &maindir, const std::string &inputdir, const std::string &outputdir, const std::string &testname, int threads);
        virtual ~RigidRegistration();

        void setMode(const std::string& mode);
        void setMethod(const std::string& method, const std::string& match, const std::string& estimation);
        void setGTfile(const std::string& gtfilepath);
        void setPointClouds(const std::string& sourcemesh, const std::string& targetmesh, int downscalestep, int totalholes, double holeradius);

        void saveParameters();

        void run();

    protected:

    private:
		MethodsData *data;
};

#endif // RIGIDREGISTRATION_H
