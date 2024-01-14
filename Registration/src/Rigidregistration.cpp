#include "RigidRegistration.h"
#include "DirHandler.h"
#include "Estimators.h"
#include "pch.h"
#include <iostream>
#include <iomanip>


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
    MethodsData::MODE mode;
    MethodsData::METHOD method;
    MethodsData::MATCH match;
    MethodsData::ESTIMATION estimation;
    data->getActiveMethod(mode, method, match, estimation);

    const PointCloud* sourcemesh = data->getSourcePointCloud();
    PointCloud* targetmesh = data->getTargetPointCloud()->Copy();
    
    unsigned int currentErrorIterations = 0;
    unsigned int currentIterations = 0;
    double currentError = initialError;
    double previousError = initialError;
    const int cout_precision = 6;
    while (currIterationWeight > 0.0 && currentError > 1e-12)
    {
        if (std::abs(currIterationWeight - minIterationWeight) < 0.0000077 || currIterationWeight < minIterationWeight)
        {
            std::cout << std::scientific << std::setprecision(cout_precision) << "Setting weight to zero: cur. weight: ";
            std::cout << currIterationWeight << " <= " << minIterationWeight << " = min. weight\n";
            currIterationWeight = 0.0;
        }

        MatchPointClouds();
        const Eigen::Affine3d transformation = estimationFunction(sourcemesh, targetmesh, tgt2src_correspondence, currIterationWeight);
        ++currentErrorIterations;
        ++currentIterations;

        previousError = currentError;
        // compute error
        currentError = RMS_Error(sourcemesh, targetmesh, tgt2src_correspondence, tgt2src_tensorCorrespondence, transformation, currIterationWeight);

        std::cout << "It. " << currentIterations << ": ";
        // the method can be stuck improving the error in the 9th decimal. If this happens, it can do it up to 200 times.
        if (previousError - currentError > 1e-10 && currentErrorIterations < 200)  
        {
            std::cout << std::scientific << std::setprecision(cout_precision) << "Error improved from " << previousError << " to " << currentError << std::endl;
            // accept transformation
            targetmesh->ApplyTransformation(transformation);
        }
        else
        {
            std::cout << std::scientific << std::setprecision(cout_precision) << "Error did not improved (" << previousError << " <= " << currentError <<"). " ;
            if (method == MethodsData::METHOD::ICP && match == MethodsData::MATCH::ICP && estimation == MethodsData::ESTIMATION::ICP)
            {
                currIterationWeight = 0.0;
                std::cout << "Stoping.\n";
            }
            else
            {
                std::cout << "Updating weight from "<< currIterationWeight << " to " ;

                currIterationWeight *= stepIterationWeight;
                std::cout << currIterationWeight << "\n";

                currentErrorIterations = 0;
            }
        }
    }

    // after the resgistration, compute the correspondent points
    distanceFunction = Point::EuclideanDistance;
    MatchPointClouds(); // updates the correspondence list, now based on euclidean distance
    currentError = RootMeanSquare(sourcemesh, targetmesh, tgt2src_correspondence);

    unsigned int i = 0;
    unsigned int correspondences = 0;
    for (unsigned int t : tgt2src_correspondence) 
    {
        if (i == t) 
        {
            correspondences++;
        }
        ++i;
    }
    std::cout << "Final error: " << currentError << " correspondences " << correspondences << std::endl;
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
        estimationFunction = Estimators::ICP_Besl;

        RMS_Error = RootMeanSquareOfTransformation;
    }
    // ICP-CTSF
    if (method == MethodsData::METHOD::ICP && match == MethodsData::MATCH::CTSF && estimation == MethodsData::ESTIMATION::ICP)
    {
        currIterationWeight = maxIterationWeight;
        minIterationWeight = 1e-6;

        distanceFunction = Point::CTSF_TensorDistance;
        estimationFunction = Estimators::ICP_Besl;

        RMS_Error = RootMeanSquareOfTransformation;
    }
    // SWC-ICP
    if ( method == MethodsData::METHOD::ICP && 
        (match == MethodsData::MATCH::ICP || match == MethodsData::MATCH::CTSF) &&
         estimation == MethodsData::ESTIMATION::SWC)
    {
        currIterationWeight = maxIterationWeight;
        minIterationWeight = 1e-6;

        distanceFunction = Point::EuclideanDistance;
        if(match == MethodsData::MATCH::CTSF)
            distanceFunction = Point::CTSF_TensorDistance;
        estimationFunction = Estimators::SWC_Akio;

        RMS_Error = RootMeanSquareSWC;

        // set correspondence list based on CTSF
        if (!data->getSourcePointCloud()->IsCTSF_DistanceListSet() || !data->getTargetPointCloud()->IsCTSF_DistanceListSet())
        {
            PRINT_ERROR("Error: Need to set the CTSF distance list before running the SWC method.");
            exit(-1);
        }
        SetTensorCorrespondenceList();
        Estimators::SetTensorCorrespondenceList(data->getSourcePointCloud(), tgt2src_tensorCorrespondence);
    }
}

void RigidRegistration::SetTensorCorrespondenceList()
{
    // building the correspondence list
    tgt2src_tensorCorrespondence.clear();

    const PointCloud* sourcemesh = data->getSourcePointCloud();
    const PointCloud* targetmesh = data->getTargetPointCloud();
    const auto& source_points = sourcemesh->GetPoints();
    const auto& target_points = targetmesh->GetPoints();
    for (unsigned int targetCounter = 0; targetCounter < target_points.size(); ++targetCounter)
    {
        const Point* tgt_pt = target_points.at(targetCounter);
        tgt2src_tensorCorrespondence.push_back(targetCounter);

        double distance = DBL_MAX;
        for (unsigned int sourceCounter = 0; sourceCounter < source_points.size(); ++sourceCounter)
        {
            const Point* src_pt = source_points.at(sourceCounter);
            double d = Point::PureCTSF_TensorDistance(src_pt, tgt_pt);
            if (d < distance)
            {
                distance = d;
                tgt2src_tensorCorrespondence.back() = sourceCounter;
            }
        }
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
    for (unsigned int targetCounter = 0; targetCounter < target_points.size(); ++targetCounter)
    {
        const Point* tgt_pt = target_points.at(targetCounter);
        tgt2src_correspondence.push_back(targetCounter);

        double distance = DBL_MAX;
        for (unsigned int sourceCounter = 0; sourceCounter < source_points.size(); ++sourceCounter)
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

    /*
    src2tgt_correspondence.clear();
    for (unsigned int sourceCounter = 0; sourceCounter < source_points.size(); ++sourceCounter)
    {
        const Point* src_pt = source_points.at(sourceCounter);
        src2tgt_correspondence.push_back(sourceCounter);

        double distance = DBL_MAX;
        for (unsigned int targetCounter = 0; targetCounter < target_points.size(); ++targetCounter)
        {
            const Point* tgt_pt = target_points.at(targetCounter);
            double d = distanceFunction(src_pt, tgt_pt, currIterationWeight);
            if (d < distance)
            {
                distance = d;
                src2tgt_correspondence.back() = targetCounter;
            }
        }
    }
    std::for_each(tgt2src_correspondence.begin(), tgt2src_correspondence.end(),
        [](const int a) 
        {
            std::cout << a << " ";
        });*/

}

double RigidRegistration::RootMeanSquareOfTransformation(const PointCloud* sourcemesh, const PointCloud* targetmesh,
    const std::vector<unsigned int>& tgt2src_correspondence,
    const std::vector<unsigned int>& tgt2src_tensorCorrespondence,
    const Eigen::Affine3d& transformation, const double weight)
 {
    const auto& source_points = sourcemesh->GetPoints();
    const auto& target_points = targetmesh->GetPoints();

    double error = 0;
    Eigen::Vector3d targetCentroid = Estimators::ComputeCenteroid(target_points);

    for (unsigned int index = 0; index < tgt2src_correspondence.size(); ++index)
    {
        Eigen::Vector3d targetPoint = target_points.at(index)->GetPosition();
        Eigen::Vector3d sourcePoint = source_points.at(tgt2src_correspondence.at(index))->GetPosition();
        targetPoint -= targetCentroid;
        targetPoint = transformation * targetPoint;
        targetPoint += targetCentroid;
        Eigen::Vector3d residual = targetPoint - sourcePoint;
        error += residual.squaredNorm();
    }
    error /= (double) tgt2src_correspondence.size();
    return std::sqrt(error);
}

double RigidRegistration::RootMeanSquareSWC(const PointCloud* sourcemesh, const PointCloud* targetmesh,
    const std::vector<unsigned int>& tgt2src_correspondence,
    const std::vector<unsigned int>& tgt2src_tensorCorrespondence,
    const Eigen::Affine3d& transformation, const double weight)
{
    const auto& source_points = sourcemesh->GetPoints();
    const auto& target_points = targetmesh->GetPoints();

    double error = 0;
    Eigen::Vector3d targetCentroid = Estimators::ComputeCenteroid(target_points);

    for (unsigned int index = 0; index < tgt2src_correspondence.size(); ++index)
    {
        Eigen::Vector3d targetPoint = target_points.at(index)->GetPosition();
        Eigen::Vector3d sourcePoint = source_points.at(tgt2src_correspondence.at(index))->GetPosition();
        if (std::abs(weight) > 0.0000077)
        {
            sourcePoint = (sourcePoint + weight * source_points.at(tgt2src_tensorCorrespondence.at(index))->GetPosition()) / (weight + 1);
        }
        targetPoint -= targetCentroid;
        targetPoint = transformation * targetPoint;
        targetPoint += targetCentroid;
        Eigen::Vector3d residual = targetPoint - sourcePoint;
        error += residual.squaredNorm();
    }

    error /= (double)tgt2src_correspondence.size();
    return std::sqrt(error);
}

double RigidRegistration::RootMeanSquare(const PointCloud* sourcemesh, const PointCloud* targetmesh,
    const std::vector<unsigned int>& tgt2src_correspondence)
{
    const auto& source_points = sourcemesh->GetPoints();
    const auto& target_points = targetmesh->GetPoints();

    double error = 0;
    int c = 0;
    Eigen::Vector3d targetCentroid = Estimators::ComputeCenteroid(target_points);

    for (unsigned int index = 0; index < tgt2src_correspondence.size(); ++index)
    {
        Eigen::Vector3d targetPoint = target_points.at(index)->GetPosition();
        Eigen::Vector3d sourcePoint = source_points.at(tgt2src_correspondence.at(index))->GetPosition();
        Eigen::Vector3d residual = targetPoint - sourcePoint;
        error += residual.squaredNorm();
    }
    error /= (double)tgt2src_correspondence.size();
    return std::sqrt(error);
}
