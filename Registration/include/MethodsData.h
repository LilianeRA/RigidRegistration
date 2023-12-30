#ifndef METHODSDATA_H
#define METHODSDATA_H

#include "PointCloud.h"

class MethodsData
{
    public:
        MethodsData(const std::string &maindir, const std::string &inputdir, const std::string &outputdir, const std::string &testname, int threads);
        virtual ~MethodsData();

        void setMode(const std::string &mode);
        void setMethod(const std::string &method, const std::string &match, const std::string &estimation);
        void setGTfile(const std::string &gtfilepath);
        void setPointClouds(const std::string &sourcemesh, const std::string &targetmesh, int downscalestep, int totalholes, double holeradius);

        void run();

		void saveParameters();
    protected:

    private:
        enum class MODE
        {
            MESHBACTH, MESHVIEW,
            VIDEOBATCH, VIDEOVIEW,
            ERROR
        };
        enum class METHOD
        {
            ICP, SPARSEICP, SWC,
            SUPER4PCS, GMM,
            ERROR
        };
        enum class MATCH
        {
            ICP, CTSF, LIEDIR,
            SUPER4PCS, GMM, LIEIND,
            ERROR
        };
        enum class ESTIMATION
        {
            ICP, SPARSEICP, SUPER4PCS, GMM,
            ERROR
        };

        MODE mode; // meshbatch meshview videobatch videoview

        METHOD method;
        MATCH match;
        ESTIMATION estimation;

        int threads = 0;
        int executionnumber = 0;
        bool useGT = false;
        double trimming = 0.0;
        std::string maindir = "";
        std::string inputdir = "";
        std::string outputdir = ""; // is also the result directory
        std::string testname = "";
        std::string gtfilepath = "";
		PointCloud *sourcemesh;
		PointCloud *targetmesh;
		int totalholes;
		double holeradius;
		std::vector<int> holesIndex; // the hole center is a point of the point cloud 

        void initInput(int downscalestep);
};

#endif // METHODSDATA_H
