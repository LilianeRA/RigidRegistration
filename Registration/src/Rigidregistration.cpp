#include "RigidRegistration.h"
#include "DirHandler.h"
#include <iostream>


RigidRegistration::RigidRegistration(const MethodsData *methodsData)
{
    //ctor
	data = methodsData;
    Setup();
}

RigidRegistration::~RigidRegistration()
{
    //dtor
}

void RigidRegistration::Run()
{
    /*MethodsData::MODE mode;
    MethodsData::METHOD method;
    MethodsData::MATCH match;
    MethodsData::ESTIMATION estimation;
    data->getActiveMethod(mode, method, match, estimation);

    const PointCloud* sourcemesh = data->getSourcePointCloud();
    const PointCloud* targetmesh = data->getTargetPointCloud();
    */
    MatchPointClouds();
}

void RigidRegistration::Setup()
{
    MethodsData::MODE mode;
    MethodsData::METHOD method;
    MethodsData::MATCH match;
    MethodsData::ESTIMATION estimation;
    data->getActiveMethod(mode, method, match, estimation);


    // original ICP
    if (method == MethodsData::METHOD::ICP && match == MethodsData::MATCH::ICP && estimation == MethodsData::ESTIMATION::ICP)
    {
        currIterationWeight = maxIterationWeight;
        minIterationWeight = 1e-6;

        distanceFunction = Point::EuclideanDistance;
    }
    // ICP-CTSF
    if (method == MethodsData::METHOD::ICP && match == MethodsData::MATCH::CTSF && estimation == MethodsData::ESTIMATION::ICP)
    {
        currIterationWeight = maxIterationWeight;
        minIterationWeight = 1e-6;

        distanceFunction = Point::CTSF_TensorDistance;
    }
}


void RigidRegistration::MatchPointClouds()
{
    // building the correspondence list
    tgt2src_correspondence.clear();

    const PointCloud* sourcemesh = data->getSourcePointCloud();
    const PointCloud* targetmesh = data->getTargetPointCloud();
    const auto& source_points = sourcemesh->GetPoints();
    const auto& target_points = targetmesh->GetPoints();
    for (int targetCounter = 0; targetCounter < target_points.size(); ++targetCounter)
    {
        const Point* tgt_pt = target_points.at(targetCounter);
        tgt2src_correspondence.push_back(targetCounter);

        double distance = DBL_MAX;
        for (int sourceCounter = 0; sourceCounter < source_points.size(); ++sourceCounter)
        {
            const Point* src_pt = source_points.at(sourceCounter);
            double d = distanceFunction(src_pt, tgt_pt, currIterationWeight);
            if (d < distance)
            {
                distance = d;
                tgt2src_correspondence.back() = sourceCounter;
            }
        }
    }
    std::for_each(tgt2src_correspondence.begin(), tgt2src_correspondence.end(),
        [](const int a) 
        {
            std::cout << a << " ";
        });

}

