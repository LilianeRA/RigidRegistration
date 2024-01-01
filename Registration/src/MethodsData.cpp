#include "MethodsData.h"
#include "DirHandler.h"
#include "Visualizer3D.h"
#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <filesystem>


MethodsData::MethodsData(const std::string &maindir,const std::string &inputdir, const std::string &outputdir, const std::string &testname, int threads)
{
    //ctor
    if(testname.empty())
    {
        std::cout<<"Error: testname empty\n";
        exit(EXIT_FAILURE);
    }
    if(threads <= 0)
    {
        std::cout<<"Warning: number of threads invalid. Setting to 4.\n";
        threads = 4;
    }

	this->maindir = maindir;
	this->inputdir = inputdir;
	this->outputdir = outputdir;
    this->testname = testname;
    this->threads = threads;

    this->mode = MODE::ERROR;
    this->method = METHOD::ERROR;
    this->match = MATCH::ERROR;
    this->estimation = ESTIMATION::ERROR;

	this->sourcemesh = nullptr;
	this->targetmesh = nullptr;
}

MethodsData::~MethodsData()
{
    //dtor
}

void MethodsData::setMode(const std::string &mode)
{
	if( mode.find("meshbatch") != std::string::npos )
    {
        this->mode = MODE::MESHBACTH;
        std::cout<<"Batch processing for mesh selected."<<std::endl;
    }
	if( mode.find("meshview") != std::string::npos )
    {
        this->mode = MODE::MESHVIEW;
        std::cout<<"Mesh visualization selected."<<std::endl;
    }
	if( mode.find("videobatch") != std::string::npos )
    {
        this->mode = MODE::VIDEOBATCH;
        std::cout<<"Batch processing for video selected."<<std::endl;
    }
	if( mode.find("videoview") != std::string::npos )
    {
        this->mode = MODE::VIDEOVIEW;
        std::cout<<"Video visualization selected."<<std::endl;
    }


    if(this->mode == MODE::ERROR)
	{
        std::cout<<"Error: mode not recognized."<<std::endl;
        exit(EXIT_FAILURE);
	}
}

void MethodsData::setMethod(const std::string &method, const std::string &match, const std::string &estimation)
{
    if(method.find("ICP")   != std::string::npos) {this->method = METHOD::ICP; std::cout<<"Method: ICP"<<std::endl;}
    if(method.find("SWC")   != std::string::npos) {this->method = METHOD::SWC; std::cout<<"Method: SWC"<<std::endl;}
    if(method.find("GMM")   != std::string::npos) {this->method = METHOD::GMM; std::cout<<"Method: GMM"<<std::endl;}
    if(method.find("SICP")  != std::string::npos) {this->method = METHOD::SPARSEICP; std::cout<<"Method: Sparse ICP"<<std::endl;}
    if(method.find("S4PCS") != std::string::npos) {this->method = METHOD::SUPER4PCS; std::cout<<"Method: Super 4PCS"<<std::endl;}

    if(match.find("ICP")    != std::string::npos) {this->match = MATCH::ICP;  std::cout<<"Match: ICP/Sparse ICP/SWC"<<std::endl;}
    if(match.find("GMM")    != std::string::npos) {this->match = MATCH::GMM;  std::cout<<"Match: GMM"<<std::endl;}
    if(match.find("CTSF")   != std::string::npos) {this->match = MATCH::CTSF; std::cout<<"Match: CTSF"<<std::endl;}
    if(match.find("S4PCS")  != std::string::npos) {this->match = MATCH::SUPER4PCS; std::cout<<"Match: Super 4PCS"<<std::endl;}
    if(match.find("LIEDIR") != std::string::npos) {this->match = MATCH::LIEDIR; std::cout<<"Match: Lie Direct"<<std::endl;}
    if(match.find("LIEIND") != std::string::npos) {this->match = MATCH::LIEIND; std::cout<<"Match: Lie Indirect"<<std::endl;}

    if(estimation.find("ICP")   != std::string::npos) {this->estimation = ESTIMATION::ICP; std::cout<<"Estimation: ICP"<<std::endl;}
    if(estimation.find("GMM")   != std::string::npos) {this->estimation = ESTIMATION::GMM; std::cout<<"Estimation: GMM"<<std::endl;}
    if(estimation.find("SICP")  != std::string::npos) {this->estimation = ESTIMATION::SPARSEICP; std::cout<<"Estimation: Sparse ICP"<<std::endl;}
    if(estimation.find("S4PCS") != std::string::npos) {this->estimation = ESTIMATION::SUPER4PCS; std::cout<<"Estimation: Super 4PCS"<<std::endl;}

    if(this->method == METHOD::ERROR)
    {
        std::cout<<"Error: method not recognized.\n";
        exit(EXIT_FAILURE);
    }
    if(this->match == MATCH::ERROR)
    {
        std::cout<<"Error: match function not recognized.\n";
        exit(EXIT_FAILURE);
    }
    if(this->estimation == ESTIMATION::ERROR)
    {
        std::cout<<"Error: estimation function not recognized.\n";
        exit(EXIT_FAILURE);
    }

}

void MethodsData::setGTfile(const std::string &gtfilepath)
{
    this->useGT = true;
    if(gtfilepath.empty())
    {
        this->useGT = false;
        this->gtfilepath = gtfilepath;
        std::cout<<"Groundtruth file not used."<<std::endl;
    }
    else
    {
        this->gtfilepath = gtfilepath;
        std::cout<<"Groundtruth file: "<<this->gtfilepath<<std::endl;
    }
}

void MethodsData::setPointClouds(const std::string &sourcemesh, const std::string &targetmesh,
		int downscalestep, int totalholes, double holeradius)
{
    if(this->mode == MODE::MESHBACTH || this->mode == MODE::MESHVIEW)
    {
		if(sourcemesh.empty() || targetmesh.empty())
		{
		    std::cout<<"Error: point cloud filename empty\n";
		    exit(EXIT_FAILURE);
		}
        if(sourcemesh.find(".ply") == std::string::npos &&
           sourcemesh.find(".dat") == std::string::npos ||
           targetmesh.find(".ply") == std::string::npos &&
           targetmesh.find(".dat") == std::string::npos )
        {
            std::cout<<"Error: input file not supported. Aborting."<<std::endl;
			std::cout<<"Source: "<<sourcemesh<<std::endl;
			std::cout<<"Target: "<<targetmesh<<std::endl;
            exit(EXIT_FAILURE);
        }
        std::string srcmesh = DirHandler::JoinPaths(this->inputdir, sourcemesh);
        std::string tgtmesh = DirHandler::JoinPaths(this->inputdir, targetmesh);

		this->sourcemesh = new PointCloud(srcmesh);
		this->targetmesh = new PointCloud(tgtmesh);
		this->totalholes = totalholes;
		this->holeradius = holeradius;
        initInput(downscalestep);
    }
    else
    {
        std::cout<<std::endl;
        std::cout<<"Warning: passing mesh file to video mode. Ignored."<<std::endl;
        std::cout<<std::endl;
    }
}

void MethodsData::saveParameters()
{
	std::ofstream paramfile;
	std::string paramname("parameters.txt");
	std::string parampath("");
	parampath = DirHandler::JoinPaths(this->outputdir, this->testname);
	if(!DirHandler::IsDirectory(parampath))
    {
        DirHandler::CreateDir(parampath);
    }
	parampath = DirHandler::JoinPaths(parampath, paramname);

	std::cout<<"Saving parameters in:"<<std::endl;
	std::cout<<parampath<<std::endl;

	paramfile.open(parampath);
	paramfile << "method=";
	if(this->method == METHOD::ERROR) paramfile << "ERROR";
	if(this->method == METHOD::ICP) paramfile << "ICP";
	if(this->method == METHOD::GMM) paramfile << "GMM";
	if(this->method == METHOD::SWC) paramfile << "SWC";
	if(this->method == METHOD::SUPER4PCS) paramfile << "S4PCS";
	if(this->method == METHOD::SPARSEICP) paramfile << "SICP";
	paramfile << "\n";
	paramfile << "match=";
	if(this->match == MATCH::ERROR) paramfile << "ERROR";
	if(this->match == MATCH::ICP) paramfile << "ICP";
	if(this->match == MATCH::GMM) paramfile << "GMM";
	if(this->match == MATCH::SUPER4PCS) paramfile << "S4PCS";
	if(this->match == MATCH::LIEDIR) paramfile << "LIEDIR";
	if(this->match == MATCH::LIEIND) paramfile << "LIEIND";
	if(this->match == MATCH::CTSF) paramfile << "CTSF";
	paramfile << "\n";
	paramfile << "estimation=";
	if(this->estimation == ESTIMATION::ERROR) paramfile << "ERROR";
	if(this->estimation == ESTIMATION::ICP) paramfile << "ICP";
	if(this->estimation == ESTIMATION::SPARSEICP) paramfile << "SICP";
	if(this->estimation == ESTIMATION::SUPER4PCS) paramfile << "S4PCS";
	if(this->estimation == ESTIMATION::GMM) paramfile << "GMM";
	paramfile << "\n";
	paramfile << "threads=" <<this->threads << "\n";
	paramfile << "trimming=" <<this->trimming << "\n";
	paramfile << "executionnumber=" <<this->executionnumber << "\n";
	paramfile << "useGT=" <<(this->useGT ? "true" : "false")<< "\n";
	paramfile << "gtfilepath=" <<(this->gtfilepath.empty() ? "---" : this->gtfilepath)<< "\n";
	paramfile << "totalholes=" <<this->totalholes << "\n";
	paramfile << "holeradius=" <<this->holeradius << "\n";
	for(unsigned int i = 0; i < this->holesIndex.size(); i++)
	{
		paramfile << "holesIndex=" <<this->holesIndex.at(i) << "\n";
	}
	paramfile.close();
}

void MethodsData::run()
{
    if (this->mode == MODE::MESHVIEW || this->mode == MODE::VIDEOVIEW)
    {
        /*WindowGLFW* glfw = new WindowGLFW(false, "View");
		std::vector<Graphics*> graphics;
		//graphics.push_back(sourcemesh->getGraphics('h'));
		graphics.push_back(sourcemesh->getGraphics('p'));
		graphics.push_back(targetmesh->getGraphics('p'));*/
        //glfw->Run(&graphics);
        Visualizer3D *window = new Visualizer3D();
        window->Run();
    }
}

void MethodsData::initInput(int downscalestep)
{
	/// if mode == video, save list of frames with skip
	/// if mode == mesh, read source/data and target/model point cloud

    if(this->mode == MODE::MESHVIEW || this->mode == MODE::MESHBACTH)
    {
        /// read ply or dat file
		sourcemesh->setType(PointCloud::TYPE::SOURCE_DATA);
		sourcemesh->setColor(Eigen::Vector3d(0.8, 0.0, 0.0));
		sourcemesh->build(downscalestep);
		/*this->holesIndex.clear();
		if(this->totalholes > 0)
		{
			if(this->totalholes >= 1)
				this->holesIndex.push_back(sourcemesh->createHole(this->holeradius, 703));
			if(this->totalholes >= 2)
				this->holesIndex.push_back(sourcemesh->createHole(this->holeradius, 880));
		}*/
		sourcemesh->saveInput(DirHandler::JoinPaths(this->maindir, this->testname));

		targetmesh->setType(PointCloud::TYPE::TARGET_MODEL);
		targetmesh->setColor(Eigen::Vector3d(0.0, 0.0, 0.0));
		targetmesh->build(downscalestep);
		targetmesh->saveInput(DirHandler::JoinPaths(this->maindir, this->testname));
    }
}