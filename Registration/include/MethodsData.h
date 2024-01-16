#ifndef METHODSDATA_H
#define METHODSDATA_H

#include "PointCloud.h"

class MethodsData
{
    public:
        enum class MODE
        {
            MESHBACTH, MESHVIEW,
            VIDEOBATCH, VIDEOVIEW,
            ERROR
        };

        enum class METHOD
        {
            ICP, SWC,
            SPARSEICP, SUPER4PCS, GMM,
            ERROR
        };
        enum class MATCH
        {
            ICP, CTSF, 
            LIEDIR, LIEIND, GONG, CALVO, LOVRIC,
            SUPER4PCS, GMM,
            ERROR
        };
        enum class ESTIMATION
        {
            ICP, SWC, 
            SPARSEICP, SUPER4PCS, GMM,
            ERROR
        };
        MethodsData(const std::string &maindir, const std::string &inputdir, const std::string &outputdir, const std::string &testname, int threads);
        virtual ~MethodsData();

        void setMode(const std::string &mode);
        void setMethod(const std::string &method, const std::string &match, const std::string &estimation, const double ctsf_percentage);
        void setGTfile(const std::string &gtfilepath);
        void SetTensorParameters(const double alphacut_degrees, const double alphaellipse_degrees, const double sigmaN);
        void setPointClouds(const std::string &sourcemesh, const std::string &targetmesh, int downscalestep, int totalholes, double holeradius);

        void getActiveMethod(MODE &mode, METHOD &method, MATCH &match, ESTIMATION &estimation) const;

        const PointCloud* getSourcePointCloud() const;
        const PointCloud* getTargetPointCloud() const;


		void saveParameters() const;
    protected:

    private:
        MODE mode = MODE::ERROR; // meshbatch meshview videobatch videoview

        METHOD method = METHOD::ERROR;
        MATCH match = MATCH::ERROR;
        ESTIMATION estimation = ESTIMATION::ERROR;

        int threads = 0;
        int executionnumber = 0;
        bool useGT = false;
        double trimming = 0.0;
        std::string maindir = "";
        std::string inputdir = "";
        std::string outputdir = ""; // is also the result directory
        std::string testname = "";
        std::string gtfilepath = "";
		PointCloud* sourcemesh;
		PointCloud *targetmesh;
		/*int totalholes;
		double holeradius;
		std::vector<int> holesIndex; // the hole center is a point of the point cloud */

        double alphacut_radians = 0.0;
        double alphaellipse_radians = 0.0;
        double sigmaN = 0.0;
        double ctsf_percentage = 0.05; // 5% 

        bool tensorParametersSeted = false;

        void initInput(int downscalestep);
};

#endif // METHODSDATA_H
