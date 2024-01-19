#include "MethodsHandler.h"
#include "DirHandler.h"
#include "CustomWindow.h"


MethodsHandler::MethodsHandler(const std::string& maindir, const std::string& inputdir, const std::string& outputdir, const std::string& testname, int threads)
{
    //ctor
	data = new MethodsData(maindir, inputdir, outputdir, testname, threads);
}

MethodsHandler::~MethodsHandler()
{
    //dtor
}

void MethodsHandler::SetMode(const std::string& mode)
{
    data->setMode(mode);
}

void MethodsHandler::SetMethod(const std::string& method, const std::string& match, const std::string& estimation, const double ctsf_percentage)
{
    data->setMethod(method, match, estimation, ctsf_percentage);
}

void MethodsHandler::SetGTfile(const std::string& gtfilepath)
{
    data->setGTfile(gtfilepath);
}

void MethodsHandler::SetPointClouds(const std::string& sourcemesh, const std::string& targetmesh, int downscalestep, int totalholes, double holeradius)
{
    data->setPointClouds(sourcemesh, targetmesh, downscalestep, totalholes, holeradius);
}

void MethodsHandler::SetTensorParameters(const double alphacut_degrees, const double alphaellipse_degrees, const double sigmaN)
{
    data->SetTensorParameters(alphacut_degrees, alphaellipse_degrees, sigmaN);
}

void MethodsHandler::SaveParameters()
{
    data->saveParameters();
}

void MethodsHandler::Run()
{
    // make sure that all the configurations are seted.
    registration = new RigidRegistration(data);

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
        window->SetRegistration(registration);
        window->SetSourcePointCloud(sourcemesh, glm::vec3(0.5, 0.0, 0.0));
        window->SetTargetPointCloud(targetmesh, glm::vec3(0.0, 0.0, 0.0));
        window->Run();
    }
    else if (mode == MethodsData::MODE::MESHBACTH)
    {
        registration->Run();
    }
}


