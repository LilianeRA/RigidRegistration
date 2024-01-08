#include "RigidRegistration.h"
#include "DirHandler.h"
#include "CustomWindow.h"


RigidRegistration::RigidRegistration(const std::string& maindir, const std::string& inputdir, const std::string& outputdir, const std::string& testname, int threads)
{
    //ctor
	data = new MethodsData(maindir, inputdir, outputdir, testname, threads);
}

RigidRegistration::~RigidRegistration()
{
    //dtor
}

void RigidRegistration::SetMode(const std::string& mode)
{
    data->setMode(mode);
}

void RigidRegistration::SetMethod(const std::string& method, const std::string& match, const std::string& estimation)
{
    data->setMethod(method, match, estimation);
}

void RigidRegistration::SetGTfile(const std::string& gtfilepath)
{
    data->setGTfile(gtfilepath);
}

void RigidRegistration::SetPointClouds(const std::string& sourcemesh, const std::string& targetmesh, int downscalestep, int totalholes, double holeradius)
{
    data->setPointClouds(sourcemesh, targetmesh, downscalestep, totalholes, holeradius);
}

void RigidRegistration::SetTensorParameters(const double alphacut, const double alphaellipse, const double sigmaN)
{
    data->SetTensorParameters(alphacut, alphaellipse, sigmaN);
}

void RigidRegistration::SaveParameters()
{
    data->saveParameters();
}

void RigidRegistration::Run()
{
    MethodsData::MODE mode;
    MethodsData::METHOD method;
    MethodsData::MATCH match;
    MethodsData::ESTIMATION estimation;
    data->getActiveMethod(mode, method, match, estimation);

    const PointCloud* sourcemesh = data->getSourcePointCloud();
    const PointCloud* targetmesh = data->getTargetPointCloud();

    if (mode == MethodsData::MODE::MESHVIEW || mode == MethodsData::MODE::VIDEOVIEW)
    {
        CustomWindow* window = new CustomWindow(false, "View");
        window->InitializeWindow();
        window->SetActiveMethod(data);
        window->SetSourcePointCloud(sourcemesh, glm::vec3(0.5, 0.0, 0.0));
        window->SetTargetPointCloud(targetmesh, glm::vec3(0.0, 0.0, 0.0));
        window->Run();
    }
    else if (mode == MethodsData::MODE::MESHBACTH)
    {
        //Iteration();
    }
}

void RigidRegistration::Iteration()
{
}

void RigidRegistration::MatchPointClouds()
{
}

void RigidRegistration::EstimateTransformation()
{
}
